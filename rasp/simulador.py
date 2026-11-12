"""
Simula o Arduino para testar o painel sem o hardware (python3 servidor.py --simular).
Imita o protocolo e a máquina de estados do firmware. O ambiente tem paredes, um
móvel fixo dentro do alcance (que a calibração deve ignorar), uma pessoa que
aparece de tempos em tempos e, às vezes, um objeto novo que fica parado.
"""
import math
import queue
import random
import threading
import time

PASSO = 2
JANELA, JANELA_BUSCA = 12, 30


class ArduinoSimulado:
    def __init__(self):
        self.saida = queue.Queue()
        self.lock = threading.Lock()
        self.inicio = time.time()
        self.estado = "DESARMADO"
        self.armado_antes_manual = False
        self.alcance, self.patrulha, self.som = 120, True, True
        self.ang_min, self.ang_max = 15, 165
        self.ang, self.dir = 90, 1
        self.v_min, self.v_max, self.reversoes = 15, 165, 0
        self.mira, self.laser_manual = 90, False
        self.t_estado = self.t_passo = self.t_mov = self.t_trk = time.time()
        self.t_liberado = 0
        self.alvo_ang = self.alvo_dist = 0
        self.ref_dist = self.ref_ang = 0
        self.t_estavel = self.t_nivel_baixo = 0
        self.dir_alvo, self.vel, self.t_alvo, self.nivel = 1, 0.0, 0, 0
        self.cand = -1
        self.j_min = self.j_max = 0
        self.grupo, self.melhor, self.falha_grupo = [], [], 0
        self.passadas_ok = self.perdas = self.sem_alvo = 0
        self.suspeito = self.checando = self.conf = False
        self.t_check = 0
        self.fundo_cal = None
        self.fundo = [0] * 181
        self.calibrado, self.cal_passe, self.cal_desarmado = False, 0, False
        self.pir = False
        self.intruso = None
        self.objeto = None                      # objeto parado novo, sem PIR
        self.t_proximo = time.time() + 6
        self.t_proximo_objeto = time.time() + 70
        self.margem = 20
        self._out("OI,TorreSentinela,4.0-sim")
        self._est()
        self._cfg()
        threading.Thread(target=self._laco, daemon=True).start()

    # interface igual à do pyserial
    def readline(self):
        try:
            return self.saida.get(timeout=1)
        except queue.Empty:
            return b""

    def write(self, dados):
        with self.lock:
            for linha in dados.decode().splitlines():
                if linha.strip():
                    self._comando(linha.strip())

    def close(self):
        pass

    # saídas
    def _out(self, linha):
        self.saida.put((linha + "\n").encode())

    def _armado(self):
        if self.estado == "MANUAL":
            return self.armado_antes_manual
        return self.estado != "DESARMADO" and not (self.estado == "CALIBRANDO" and self.cal_desarmado)

    def _est(self):
        self._out(f"EST,{self.estado},{int(self._armado())}")

    def _cfg(self):
        self._out(f"CFG,{self.alcance},{int(self.patrulha)},{int(self.som)},{self.ang_min},{self.ang_max},{self.margem}")

    def _repouso(self):
        return (self.ang_min + self.ang_max) // 2

    def _nivel(self, n):
        if n != self.nivel:
            self.nivel = n
            self._out(f"NIVEL,{n}")

    def _mudar(self, novo):
        self.estado, self.t_estado, self.cand = novo, time.time(), -1
        if novo in ("DESARMADO", "ARMANDO", "VIGIANDO"):
            self.ang = self._repouso()
        if novo == "VARRENDO":
            self.t_mov = time.time()
        if novo != "TRAVADO":
            self._nivel(0)
        self._est()

    def _varrer_de(self, centro, janela):
        if janela:
            self.v_min, self.v_max, self.reversoes = max(self.ang_min, centro - 30), min(self.ang_max, centro + 30), 3
        else:
            self.v_min, self.v_max, self.reversoes = self.ang_min, self.ang_max, 0
        self.ang = max(self.v_min, min(self.v_max, centro))

    def _comecar_varrer(self):
        self._mudar("VARRENDO")
        self._varrer_de(self.ang, False)

    def _calibrar(self, voltar_desarmado):
        self.cal_desarmado, self.cal_passe = voltar_desarmado, 0
        self.fundo = [0] * 181
        self.calibrado = False
        self._mudar("CALIBRANDO")
        self.ang, self.dir = self.ang_min, 1
        self.t_liberado = time.time() + 0.5

    # comandos do Rasp
    def _comando(self, linha):
        cmd, _, arg = linha.partition(",")
        v = int(arg.split(",")[0]) if arg.split(",")[0].lstrip("-").isdigit() else 0
        if cmd == "PING":
            self._out(f"PONG,{int(time.time() - self.inicio)},600,{int(self.calibrado)},4,{int(self.pir)}")
        elif cmd == "ARMAR" and (self.estado in ("DESARMADO", "MANUAL") or
                                 (self.estado == "CALIBRANDO" and self.cal_desarmado)):
            self._mudar("ARMANDO")
        elif cmd == "DESARMAR":
            self._mudar("DESARMADO")
        elif cmd == "CALIBRAR":
            if self.estado == "DESARMADO":
                self._calibrar(True)
            elif self.estado in ("VIGIANDO", "VARRENDO"):
                self._calibrar(False)
            else:
                self._out("MSG,calibracao so com a torre desarmada ou vigiando")
        elif cmd == "MODO" and arg == "MANUAL" and self.estado != "MANUAL":
            self.armado_antes_manual = self._armado()
            self.mira, self.laser_manual = self.ang, False
            self._mudar("MANUAL")
        elif cmd == "MODO" and arg == "AUTO" and self.estado == "MANUAL":
            self._mudar("VIGIANDO" if self.armado_antes_manual else "DESARMADO")
        elif cmd == "MIRA" and self.estado == "MANUAL":
            self.mira = max(0, min(180, v))
        elif cmd == "LASER" and self.estado == "MANUAL":
            self.laser_manual = v != 0
        elif cmd == "ALCANCE":
            self.alcance = max(10, min(300, v)); self._cfg()
        elif cmd == "PATRULHA":
            self.patrulha = v != 0; self._cfg()
        elif cmd == "SOM":
            self.som = v != 0; self._cfg()
        elif cmd == "MARGEM":
            self.margem = max(5, min(80, v)); self._cfg()
        elif cmd == "VARRE":
            p = arg.split(",")
            mn = max(0, min(170, int(p[0])))
            mx = max(mn + 10, min(180, int(p[1]))) if len(p) > 1 else self.ang_max
            self.ang_min, self.ang_max = mn, mx
            self._cfg()
            if self.estado == "CALIBRANDO":
                self._calibrar(self.cal_desarmado)
            elif self.estado == "VARRENDO":
                self._varrer_de(self.ang, False)
            elif self.estado in ("DESARMADO", "VIGIANDO", "ARMANDO"):
                self.ang = self._repouso()
        elif cmd == "STATUS":
            self._est(); self._cfg()

    # "física" do ambiente
    @staticmethod
    def _parede(a):
        d = 210 + 45 * math.sin(math.radians(a) * 2.3) + 25 * math.cos(math.radians(a) * 5)
        if 108 <= a <= 124:          # móvel fixo dentro do alcance
            d = 72 + (a - 116) ** 2 * 0.15
        return d

    def _ler(self, a):
        if random.random() < 0.04:
            return -1
        i, o = self.intruso, self.objeto
        if i and abs(a - i["ang"]) <= 7 and i["dist"] < 400:
            return max(3, int(i["dist"] + random.gauss(0, 2)))
        if o and abs(a - o["ang"]) <= 5:
            return int(o["dist"] + random.gauss(0, 1.5))
        return int(self._parede(a) + random.gauss(0, 3))

    def _ambiente(self, agora, dt):
        i = self.intruso
        if i is None and agora >= self.t_proximo:
            self.intruso = {"ang": random.uniform(30, 150), "dist": 260.0,
                            "vang": random.uniform(-5, 5), "vdist": -random.uniform(25, 45),
                            "parar": random.uniform(30, 70), "fim": agora + random.uniform(14, 22),
                            "saindo": False}
        elif i:
            i["vang"] = max(-8, min(8, i["vang"] + random.uniform(-0.6, 0.6)))
            i["ang"] += i["vang"] * dt
            if not 15 < i["ang"] < 165:
                i["vang"] *= -1
                i["ang"] = max(15, min(165, i["ang"]))
            i["dist"] += i["vdist"] * dt
            if not i["saindo"] and i["dist"] <= i["parar"]:
                i["vdist"] = 0
            if not i["saindo"] and agora > i["fim"]:
                i["saindo"], i["vdist"] = True, 45
            if i["saindo"] and i["dist"] > 330:
                self.intruso = None
                self.t_proximo = agora + random.uniform(35, 55)
        # objeto parado que surge sem ninguém (ex.: cadeira arrastada) -> alarme falso
        if self.objeto is None and agora >= self.t_proximo_objeto and self.intruso is None:
            self.objeto = {"ang": random.choice([40, 44, 48]), "dist": random.uniform(55, 85), "fim": agora + 60}
        elif self.objeto and agora > self.objeto["fim"]:
            self.objeto = None
            self.t_proximo_objeto = agora + random.uniform(40, 90)
            if self.fundo_cal:                   # tirou o objeto: o mapa volta ao normal
                self.fundo = list(self.fundo_cal)

    def _intruso_em(self, a, d, alc):
        if d <= 0 or d > alc:
            return False
        k = max(0, min(180, a)) // 2 * 2
        refs = [self.fundo[j] for j in (k - 2, k, k + 2) if 0 <= j <= 180 and self.fundo[j]]
        if not self.calibrado or not refs:
            return True
        b = min(refs)
        return d < b - max(self.margem, b // 8)

    # laço principal (imita o loop() do Arduino)
    def _laco(self):
        anterior = time.time()
        while True:
            time.sleep(0.02)
            agora = time.time()
            with self.lock:
                self._tick(agora, agora - anterior)
            anterior = agora

    def _tick(self, agora, dt):
        self._ambiente(agora, dt)
        # o PIR gira com a torre: só vale com ela parada
        parada = self.estado in ("DESARMADO", "VIGIANDO") or self.checando
        pir = parada and self.intruso is not None and self.intruso["dist"] < 300
        if pir != self.pir:
            self.pir = pir
            self._out(f"PIR,{int(pir)}")

        e = self.estado
        if e == "DESARMADO" and agora - self.t_passo >= 0.5:
            self.t_passo = agora
            self._out(f"R,{self.ang},{self._ler(self.ang)}")
        elif e == "ARMANDO" and agora - self.t_estado >= 5:
            self._calibrar(False)
        elif e == "CALIBRANDO" and agora >= self.t_liberado and agora - self.t_passo >= 0.06:
            self.t_passo = agora
            self._passo_calibracao()
        elif e == "VIGIANDO" and (self.patrulha or self.pir):
            self._comecar_varrer()
        elif e == "VARRENDO":
            if self.pir:
                self.t_mov = agora
            if not self.patrulha and agora - self.t_mov > 15:
                self._mudar("VIGIANDO")
            elif agora - self.t_passo >= (0.035 if self.cand > 0 else 0.06):
                self.t_passo = agora
                self._varrer()
        elif e == "TRAVADO":
            if agora - self.t_passo >= 0.045:
                self.t_passo = agora
                self._rastrear(agora)
            if self.estado == "TRAVADO" and not self.suspeito:
                n = 2 if (self.alvo_dist <= self.alcance / 2 or (self.vel < -25 and self.alvo_dist < self.alcance * .75)) else 1
                if n < self.nivel:
                    self.t_nivel_baixo = self.t_nivel_baixo or agora
                    if agora - self.t_nivel_baixo < 2:
                        n = self.nivel
                if n >= self.nivel:
                    self.t_nivel_baixo = 0
                self._nivel(n)
        elif e == "MANUAL" and agora - self.t_passo >= 0.12:
            self.t_passo = agora
            self._out(f"R,{self.mira},{self._ler(self.mira)}")

    def _passo_calibracao(self):
        d = self._ler(self.ang)
        self._out(f"R,{self.ang},{d}")
        k = self.ang // 2 * 2
        if d > 0:
            v = min(d, 510) // 2 * 2
            self.fundo[k] = min(self.fundo[k], v) if self.fundo[k] else v
        self.ang += self.dir * PASSO
        if self.ang > self.ang_max:
            self.ang, self.dir, self.cal_passe = self.ang_max, -1, 1
        elif self.ang < self.ang_min:
            self.calibrado = True
            self.fundo_cal = list(self.fundo)
            validos = 0
            for a in range(self.ang_min & ~1, self.ang_max + 1, 2):
                self._out(f"BASE,{a},{self.fundo[a] or -1}")
                validos += 1 if self.fundo[a] else 0
            self._out(f"BASE_OK,{validos}")
            if self.cal_desarmado:
                self._mudar("DESARMADO")
            elif self.patrulha:
                self._comecar_varrer()
            else:
                self._mudar("VIGIANDO")

    def _janela(self, centro, meia):
        self.j_min, self.j_max = max(self.ang_min, centro - meia), min(self.ang_max, centro + meia)

    def _travar(self, ang, d):
        agora = time.time()
        self.alvo_ang, self.alvo_dist, self.vel, self.t_alvo, self.dir_alvo = ang, d, 0.0, agora, self.dir
        self.ref_dist, self.ref_ang, self.t_estavel = d, ang, agora
        self.suspeito, self.checando, self.conf = True, False, False
        self.passadas_ok = self.perdas = 0
        self.grupo, self.melhor, self.falha_grupo = [(ang, d)], [], 0
        self._janela(ang, JANELA)
        self._mudar("TRAVADO")
        self._out(f"SUSP,{ang},{d}")

    def _varrer(self):
        d = self._ler(self.ang)
        self._out(f"R,{self.ang},{d}")
        perto = self._intruso_em(self.ang, d, self.alcance)
        if self.cand > 0:                        # segunda leitura: confirma ou descarta
            c, self.cand = self.cand, -1
            if perto and abs(d - c) <= 25:
                self._travar(self.ang, (d + c) // 2)
                return
        elif perto:
            self.cand = d
            return
        self.ang += self.dir * PASSO
        if self.ang >= self.v_max:
            self.ang, self.dir = self.v_max, -1
            self._fim_curso()
        elif self.ang <= self.v_min:
            self.ang, self.dir = self.v_min, 1
            self._fim_curso()

    def _fim_curso(self):
        if self.reversoes > 0:
            self.reversoes -= 1
            if self.reversoes == 0:
                self.v_min, self.v_max = self.ang_min, self.ang_max

    def _perdeu(self):
        era_suspeito = self.suspeito
        self._out("PERDIDO")
        self._mudar("VARRENDO")
        self.dir = self.dir_alvo
        self._varrer_de(self.alvo_ang, not era_suspeito)

    def _absorver(self):
        v = min(self.alvo_dist + 2, 510) // 2 * 2
        k = self.alvo_ang // 2 * 2
        for j in range(k - 12, k + 13, 2):
            if 0 <= j <= 180 and (self.fundo[j] == 0 or self.fundo[j] > v):
                self.fundo[j] = v
        self.calibrado = True
        self._out(f"IGNORADO,{self.alvo_ang},{self.alvo_dist}")
        self._mudar("VARRENDO")
        self._varrer_de(self.alvo_ang, False)

    def _fechar_grupo(self):
        if len(self.grupo) > len(self.melhor):
            self.melhor = self.grupo
        self.grupo, self.falha_grupo = [], 0

    def _fim_passada(self, agora):
        g, self.melhor = self.melhor, []
        if len(g) < 2:
            self.perdas += 1
            if self.perdas >= (2 if self.suspeito else 3):
                self._perdeu()
            else:
                self._janela(self.alvo_ang, JANELA_BUSCA)
            return
        self.perdas = 0
        a = sum(p[0] for p in g) // len(g)
        d = sum(p[1] for p in g) // len(g)
        if self.passadas_ok and agora - self.t_alvo >= 0.2:
            inst = max(-300, min(300, (d - self.alvo_dist) / (agora - self.t_alvo)))
            self.vel = self.vel * 0.5 + inst * 0.5
        if abs(a - self.alvo_ang) >= 4:
            self.dir_alvo = 1 if a > self.alvo_ang else -1
        self.alvo_ang, self.alvo_dist, self.t_alvo = a, d, agora
        self.passadas_ok += 1
        if self.suspeito and (len(g) >= 4 or self.passadas_ok >= 2):
            self.suspeito = False
            self._out(f"ALVO,{a},{d}")
        if abs(d - self.ref_dist) > 10 or abs(a - self.ref_ang) > 8:
            self.ref_dist, self.ref_ang, self.t_estavel = d, a, agora
        elif not self.suspeito and agora - self.t_estavel > 20:
            self.checando, self.t_check, self.sem_alvo, self.ang = True, agora, 0, a
        self._janela(a, max(JANELA, len(g) + 8))
        self._out(f"TRK,{a},{d},{int(self.vel)},{len(g) * PASSO}")

    def _rastrear(self, agora):
        d = self._ler(self.ang)
        self._out(f"R,{self.ang},{d}")
        perto = self._intruso_em(self.ang, d, self.alcance + 15)
        if self.checando:                        # parada no alvo esperando o PIR
            self.sem_alvo = 0 if perto else self.sem_alvo + 1
            if self.sem_alvo >= 6:
                self.checando, self.t_estavel = False, agora
            elif self.pir and agora - self.t_check > 4:
                self.checando, self.t_estavel = False, agora
                if not self.conf:
                    self.conf = True
                    self._out("CONF")
            elif agora - self.t_check > 12:
                self._absorver()
            return
        if perto:
            self.grupo.append((self.ang, d))
            self.falha_grupo = 0
        elif self.grupo:
            self.falha_grupo += 1
            if self.falha_grupo >= 2:
                self._fechar_grupo()
        self.ang += self.dir * PASSO
        if self.ang > self.j_max or self.ang < self.j_min:
            self.dir = -1 if self.ang > self.j_max else 1
            self._fechar_grupo()
            self._fim_passada(agora)
            if self.estado != "TRAVADO" or self.checando:
                return
            self.ang = max(self.j_min, min(self.j_max, self.ang))
