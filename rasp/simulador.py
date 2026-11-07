"""
Simula o Arduino para testar o painel sem o hardware (python3 servidor.py --simular).
Imita o protocolo e a máquina de estados do firmware v2. O ambiente tem paredes,
um móvel fixo dentro do alcance (que a calibração deve ignorar), uma pessoa que
aparece de tempos em tempos e, às vezes, um objeto parado novo (gera alarme falso).
"""
import math
import queue
import random
import threading
import time

PASSO = 2
N_BUSCA = 7


class ArduinoSimulado:
    def __init__(self):
        self.saida = queue.Queue()
        self.lock = threading.Lock()
        self.inicio = time.time()
        self.estado = "DESARMADO"
        self.armado_antes_manual = False
        self.alcance, self.patrulha, self.som = 100, False, True
        self.ang_min, self.ang_max = 15, 165
        self.ang, self.dir = 90, 1
        self.v_min, self.v_max, self.reversoes = 15, 165, 0
        self.mira, self.laser_manual = 90, False
        self.t_estado = self.t_passo = self.t_mov = self.t_trk = time.time()
        self.t_liberado = 0
        self.alvo_ang = self.alvo_dist = self.falhas = self.idx = 0
        self.ref_dist = self.ref_ang = 0
        self.t_estavel = self.t_nivel_baixo = 0
        self.dir_alvo, self.vel, self.t_alvo, self.nivel = 1, 0.0, 0, 0
        self.confirmando = False
        self.fundo = [0] * 181
        self.calibrado, self.cal_passe, self.cal_desarmado = False, 0, False
        self.pir = False
        self.intruso = None
        self.objeto = None                      # objeto parado novo, sem PIR
        self.t_proximo = time.time() + 6
        self.t_proximo_objeto = time.time() + 70
        self.margem = 20
        self._out("OI,TorreSentinela,3.1-sim")
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
        self.estado, self.t_estado, self.confirmando = novo, time.time(), False
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
            self._out(f"PONG,{int(time.time() - self.inicio)},1450,{int(self.calibrado)}")
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
            self.objeto = {"ang": random.choice([40, 44, 48]), "dist": random.uniform(55, 85), "fim": agora + 45}
        elif self.objeto and agora > self.objeto["fim"]:
            self.objeto = None
            self.t_proximo_objeto = agora + random.uniform(40, 90)

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
        pir = self.intruso is not None and self.intruso["dist"] < 300
        if pir != self.pir:
            self.pir = pir
            self._out(f"PIR,{int(pir)}")

        e = self.estado
        if e == "ARMANDO" and agora - self.t_estado >= 5:
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
            elif agora - self.t_passo >= 0.06:
                self.t_passo = agora
                self._varrer()
        elif e == "TRAVADO":
            if agora - self.t_passo >= 0.08:
                self.t_passo = agora
                self._rastrear(agora)
            if self.estado == "TRAVADO":
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
            self.fundo[k] = max(self.fundo[k], min(d, 510) // 2 * 2)
        self.ang += self.dir * PASSO
        if self.ang > self.ang_max:
            self.ang, self.dir, self.cal_passe = self.ang_max, -1, 1
        elif self.ang < self.ang_min:
            self.calibrado = True
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

    def _varrer(self):
        d = self._ler(self.ang)
        self._out(f"R,{self.ang},{d}")
        if self._intruso_em(self.ang, d, self.alcance):
            if self.confirmando:
                self.alvo_ang, self.alvo_dist, self.falhas, self.idx = self.ang, d, 0, 0
                self.vel, self.t_alvo, self.dir_alvo = 0.0, time.time(), self.dir
                self.ref_dist, self.ref_ang, self.t_estavel = d, self.ang, time.time()
                self.conf = False
                self._mudar("TRAVADO")
                self._out(f"ALVO,{self.ang},{d}")
                return
            self.confirmando = True
            return
        self.confirmando = False
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

    def _offset(self, k):
        if k == 0:
            return 0
        mag = ((k + 1) // 2) * 4
        return (self.dir_alvo if k % 2 == 1 else -self.dir_alvo) * mag

    def _rastrear(self, agora):
        d = self._ler(self.ang)
        self._out(f"R,{self.ang},{d}")
        if self._intruso_em(self.ang, d, self.alcance + 15):
            dt = agora - self.t_alvo
            if dt > 0.05:
                inst = max(-300, min(300, (d - self.alvo_dist) / dt))
                self.vel = self.vel * 0.6 + inst * 0.4
            if self.ang != self.alvo_ang:
                self.dir_alvo = 1 if self.ang > self.alvo_ang else -1
            self.alvo_ang, self.alvo_dist, self.t_alvo, self.falhas, self.idx = self.ang, d, agora, 0, 0
            if abs(d - self.ref_dist) > 5 or abs(self.ang - self.ref_ang) > 4 or self.pir:
                self.ref_dist, self.ref_ang, self.t_estavel = d, self.ang, agora
            elif agora - self.t_estavel > 30:
                v = min(d + 2, 510) // 2 * 2
                k = self.ang // 2 * 2
                for j in range(k - 4, k + 5, 2):
                    if 0 <= j <= 180 and (self.fundo[j] == 0 or self.fundo[j] > v):
                        self.fundo[j] = v
                self.calibrado = True
                self._out(f"IGNORADO,{self.ang},{d}")
                self._mudar("VARRENDO")
                self._varrer_de(self.ang, False)
                return
            if self.pir and not getattr(self, "conf", False):
                self.conf = True
                self._out("CONF")
            if agora - self.t_trk >= 0.25:
                self.t_trk = agora
                self._out(f"TRK,{self.ang},{d},{int(self.vel)}")
            return
        self.falhas += 1
        if self.falhas < 2:
            return
        self.idx += 1
        if self.idx >= N_BUSCA:
            self._out("PERDIDO")
            self._mudar("VARRENDO")
            self.dir = self.dir_alvo
            self._varrer_de(self.alvo_ang, True)
            return
        self.ang = max(self.ang_min, min(self.ang_max, self.alvo_ang + self._offset(self.idx)))
