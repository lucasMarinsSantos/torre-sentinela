#!/usr/bin/env python3
"""
Torre Sentinela - servidor do Raspberry Pi

Recebe as leituras do Arduino pela USB, guarda eventos e incidentes no SQLite
e serve o painel web (http://<ip-do-rasp>:5000).

    python3 servidor.py              # com o Arduino na USB
    python3 servidor.py --simular    # sem o Arduino, com dados de teste
"""
import argparse
import collections
import glob
import json
import os
import queue
import sqlite3
import statistics
import threading
import time
from datetime import datetime, timedelta

from flask import Flask, Response, jsonify, request, send_from_directory

BASE = os.path.dirname(os.path.abspath(__file__))
DB_PATH = os.path.join(BASE, "torre.db")

FIM_INCIDENTE_S = 30          # incidente acaba após 30 s sem PIR e sem alvo
SEM_RESPOSTA_S = 20           # sem resposta do Arduino por 20 s: reconecta
FALSOS_PARA_AGIR = 3          # alarmes falsos no mesmo setor em 1 hora
AGENDA_PADRAO = {"ativa": False, "armar": "23:00", "desarmar": "07:00", "dias": [0, 1, 2, 3, 4, 5, 6]}
CLASSES = {"pessoa": "Intruso", "falso": "Objeto parado", "movimento": "Movimento fora do alcance",
           "indefinido": "Indefinido"}


def log(msg):
    print(f"[{datetime.now():%H:%M:%S}] {msg}", flush=True)


def carregar_config_env():
    """Lê config.env (CHAVE=valor) sem sobrescrever variáveis já definidas."""
    caminho = os.path.join(BASE, "config.env")
    if not os.path.exists(caminho):
        return
    with open(caminho, encoding="utf-8") as f:
        for linha in f:
            linha = linha.strip()
            if not linha or linha.startswith("#") or "=" not in linha:
                continue
            chave, valor = linha.split("=", 1)
            os.environ.setdefault(chave.strip(), valor.strip().strip('"').strip("'"))


# Banco
def db():
    conn = sqlite3.connect(DB_PATH, timeout=5)
    conn.row_factory = sqlite3.Row
    return conn


def iniciar_banco():
    with db() as c:
        c.executescript("""
            CREATE TABLE IF NOT EXISTS eventos (
                id INTEGER PRIMARY KEY AUTOINCREMENT, ts TEXT NOT NULL,
                tipo TEXT NOT NULL, detalhe TEXT, ang INTEGER, dist INTEGER);
            CREATE INDEX IF NOT EXISTS idx_eventos_ts ON eventos(ts);
            CREATE TABLE IF NOT EXISTS incidentes (
                id INTEGER PRIMARY KEY AUTOINCREMENT, inicio TEXT NOT NULL, fim TEXT NOT NULL,
                duracao INTEGER, classe TEXT, motivo TEXT, min_dist INTEGER,
                ang_min INTEGER, ang_max INTEGER, travamentos INTEGER, pir INTEGER, vel_max INTEGER,
                trajeto TEXT);
            CREATE INDEX IF NOT EXISTS idx_incidentes_inicio ON incidentes(inicio);
            CREATE TABLE IF NOT EXISTS config (chave TEXT PRIMARY KEY, valor TEXT);
        """)


def cfg_get(chave, padrao):
    with db() as c:
        row = c.execute("SELECT valor FROM config WHERE chave=?", (chave,)).fetchone()
    return json.loads(row["valor"]) if row else padrao


def cfg_set(chave, valor):
    with db() as c:
        c.execute("INSERT OR REPLACE INTO config VALUES (?, ?)", (chave, json.dumps(valor)))


def agora_iso():
    return datetime.now().isoformat(timespec="seconds")


def hhmm_valido(s):
    try:
        datetime.strptime(s, "%H:%M")
        return True
    except (TypeError, ValueError):
        return False


# Incidentes
class Incidentes:
    """Junta PIR, travamentos e trajeto num incidente e classifica."""

    def __init__(self, torre):
        self.t = torre
        self.lock = threading.Lock()
        self.atual = None
        self.falsos_recentes = []        # (timestamp, ângulo central)
        self.ultima_sugestao = 0
        threading.Thread(target=self._laco, daemon=True).start()

    def _abrir(self):
        if self.atual is None:
            self.atual = {"inicio": time.time(), "ultimo": time.time(), "pir": False, "travamentos": 0,
                          "pontos": [], "vel_max": 0}
            self.t.publicar({"t": "incidente", "aberto": True})
        return self.atual

    def resumo(self):
        with self.lock:
            a = self.atual
            if not a:
                return None
            ds = [p[2] for p in a["pontos"]]
            return {"desde": datetime.fromtimestamp(a["inicio"]).isoformat(timespec="seconds"),
                    "segundos": int(time.time() - a["inicio"]), "pir": a["pir"],
                    "travamentos": a["travamentos"], "min_dist": min(ds) if ds else None}

    def pir(self, ativo):
        if not (ativo and self.t.s["armado"]):
            return
        with self.lock:
            inc = self._abrir()
            inc["pir"] = True
            inc["ultimo"] = time.time()

    def alvo(self, ang, dist):
        if not self.t.s["armado"]:
            return
        with self.lock:
            inc = self._abrir()
            inc["travamentos"] += 1
            inc["ultimo"] = time.time()
            inc["pontos"].append((time.time(), ang, dist))

    def rastreio(self, ang, dist, vel):
        with self.lock:
            inc = self.atual
            if not inc:
                return
            inc["ultimo"] = time.time()
            if len(inc["pontos"]) < 400:
                inc["pontos"].append((time.time(), ang, dist))
            inc["vel_max"] = max(inc["vel_max"], -vel)   # velocidade de aproximação

    def desarmou(self):
        self.fechar("desarmada")

    def sirene(self):
        with self.lock:
            if not self.atual:
                return True
            if self.atual.get("sirene"):
                return False
            self.atual["sirene"] = True
            return True

    def ignorado(self):
        with self.lock:
            if self.atual:
                self.atual["ignorado"] = True

    def confirmado(self, ang, dist):
        if not self.t.s["armado"]:
            return
        with self.lock:
            inc = self._abrir()
            inc["pir"] = True
            inc["ultimo"] = time.time()

    def _laco(self):
        while True:
            time.sleep(2)
            with self.lock:
                inc = self.atual
                quieto = inc and time.time() - inc["ultimo"] > FIM_INCIDENTE_S
            if quieto and not self.t.s["pir"] and self.t.s["estado"] != "TRAVADO":
                self.fechar()

    @staticmethod
    def classificar(inc):
        if inc["travamentos"] == 0:
            return "movimento", "O PIR viu movimento, mas nada entrou no alcance da torre."
        angs = [p[1] for p in inc["pontos"]]
        ds = [p[2] for p in inc["pontos"]]
        span_ang = max(angs) - min(angs)
        span_d = max(ds) - min(ds)
        desvio = statistics.pstdev(ds) if len(ds) > 1 else 0
        movel = span_ang >= 14 or span_d >= 25 or inc["vel_max"] >= 20
        if movel:
            return "pessoa", f"O alvo se moveu dentro do alcance ({span_ang}° e {span_d} cm)."
        if inc["pir"]:
            return "pessoa", "Alvo parado, mas o PIR confirmou movimento."
        if inc.get("ignorado"):
            return "falso", "Apareceu algo novo que ficou parado e o PIR não confirmou: entrou no mapa."
        if len(ds) < 4 and desvio < 6:
            return "indefinido", "Alvo visto por pouco tempo, sempre no mesmo lugar."
        return "indefinido", "Alvo parado dentro do alcance, sem confirmação do PIR."

    def fechar(self, motivo_fim=None):
        with self.lock:
            inc, self.atual = self.atual, None
        if not inc:
            return
        classe, motivo = self.classificar(inc)
        angs = [p[1] for p in inc["pontos"]] or [None]
        ds = [p[2] for p in inc["pontos"]] or [None]
        dur = int(inc["ultimo"] - inc["inicio"])
        reg = {
            "inicio": datetime.fromtimestamp(inc["inicio"]).isoformat(timespec="seconds"),
            "fim": datetime.fromtimestamp(inc["ultimo"]).isoformat(timespec="seconds"),
            "duracao": dur, "classe": classe, "motivo": motivo,
            "min_dist": min(d for d in ds if d is not None) if ds[0] is not None else None,
            "ang_min": min(a for a in angs if a is not None) if angs[0] is not None else None,
            "ang_max": max(a for a in angs if a is not None) if angs[0] is not None else None,
            "travamentos": inc["travamentos"], "pir": int(inc["pir"]), "vel_max": int(inc["vel_max"]),
            "trajeto": json.dumps([[p[1], p[2]] for p in inc["pontos"][::max(1, len(inc["pontos"]) // 60)]]),
        }
        with db() as c:
            cur = c.execute(
                "INSERT INTO incidentes (inicio, fim, duracao, classe, motivo, min_dist, ang_min, ang_max, "
                "travamentos, pir, vel_max, trajeto) VALUES (:inicio,:fim,:duracao,:classe,:motivo,:min_dist,"
                ":ang_min,:ang_max,:travamentos,:pir,:vel_max,:trajeto)", reg)
            reg["id"] = cur.lastrowid
        reg["trajeto"] = json.loads(reg["trajeto"])
        log(f"Incidente {reg['id']}: {CLASSES[classe]} ({dur} s) - {motivo}")
        self.t.publicar({"t": "incidente", "aberto": False, "incidente": reg})
        self.t.registrar("incidente", f"{CLASSES[classe]}: {dur} s" +
                         (f", mais perto {reg['min_dist']} cm" if reg["min_dist"] else ""))

        if classe == "falso" and reg["ang_min"] is not None:
            self._aprender_falso((reg["ang_min"] + reg["ang_max"]) // 2)

    def _aprender_falso(self, ang):
        # varios alarmes falsos no mesmo setor: o mapa esta desatualizado
        agora = time.time()
        self.falsos_recentes = [(t, a) for t, a in self.falsos_recentes if agora - t < 3600] + [(agora, ang)]
        vizinhos = [a for t, a in self.falsos_recentes if abs(a - ang) <= 10]
        if len(vizinhos) < FALSOS_PARA_AGIR:
            return
        setor = f"{min(vizinhos)}° a {max(vizinhos)}°"
        self.falsos_recentes = [(t, a) for t, a in self.falsos_recentes if abs(a - ang) > 10]
        s = self.t.s
        if cfg_get("auto_recal", True) and s["estado"] in ("VIGIANDO", "VARRENDO") and not s["pir"]:
            if self.t.enviar("CALIBRAR"):
                texto = f"Recalibrando sozinha: o setor {setor} gerou {len(vizinhos)} alarmes falsos em 1 hora."
                self.t.registrar("sugestao", texto)
                return
        if agora - self.ultima_sugestao > 3600:
            self.ultima_sugestao = agora
            texto = f"O setor {setor} gerou {len(vizinhos)} alarmes falsos. Recalibre com a área vazia."
            self.t._atualizar(sugestao=texto)
            self.t.registrar("sugestao", texto)


# Torre
class Torre:
    """Guarda o estado atual e conversa com o Arduino."""

    def __init__(self, simular=False):
        self.simular = simular
        self.lock = threading.Lock()
        self.serial_lock = threading.Lock()
        self.ser = None
        self.ultimo_rx = time.time()
        self.diag = collections.deque(maxlen=40000)   # ultimas mensagens da serial
        self.t_porta = 0                              # quando a porta serial foi aberta
        self.inscritos = []
        self.radar = [None] * 181          # (dist, timestamp) por ângulo
        self.fundo_novo = {}
        fundo = cfg_get("fundo", None)
        self.s = {
            "conectado": False, "versao": None, "porta": None, "simulado": simular,
            "estado": "DESCONECTADO", "armado": False, "pir": False, "nivel": 0,
            "alcance": cfg_get("alcance", 120), "patrulha": cfg_get("patrulha", True),
            "som": cfg_get("som", True), "ang_min": cfg_get("ang_min", 15), "ang_max": cfg_get("ang_max", 165),
            "laser": False, "angulo": 90, "dist": -1, "alvo": None,
            "calibrado": False, "fundo": fundo, "fundo_ts": cfg_get("fundo_ts", None),
            "uptime": None, "ram": None, "sugestao": None, "margem": cfg_get("margem", 20),
            "sem_eco": None, "pir_bruto": None, "reinicios": 0,
        }
        self.incidentes = Incidentes(self)

    # publicação para o painel (SSE)
    def inscrever(self):
        q = queue.Queue(maxsize=500)
        with self.lock:
            self.inscritos.append(q)
        return q

    def desinscrever(self, q):
        with self.lock:
            if q in self.inscritos:
                self.inscritos.remove(q)

    def publicar(self, evento):
        with self.lock:
            for q in self.inscritos:
                try:
                    q.put_nowait(evento)
                except queue.Full:
                    pass

    def registrar(self, tipo, detalhe="", ang=None, dist=None):
        ts = agora_iso()
        with db() as c:
            c.execute("INSERT INTO eventos (ts, tipo, detalhe, ang, dist) VALUES (?,?,?,?,?)",
                      (ts, tipo, detalhe, ang, dist))
        self.publicar({"t": "evento", "ts": ts, "tipo": tipo, "detalhe": detalhe, "ang": ang, "dist": dist})

    def contagem_hoje(self):
        hoje = datetime.now().strftime("%Y-%m-%d")
        with db() as c:
            rows = c.execute("SELECT classe, COUNT(*) n FROM incidentes WHERE inicio >= ? GROUP BY classe",
                             (hoje,)).fetchall()
        return {r["classe"]: r["n"] for r in rows}

    def snapshot(self):
        agora = time.time()
        with self.lock:
            s = dict(self.s)
            s["radar"] = [None if r is None else [r[0], round(agora - r[1], 2)] for r in self.radar]
        s["hoje"] = self.contagem_hoje()
        s["incidente"] = self.incidentes.resumo()
        s["agenda"] = cfg_get("agenda", AGENDA_PADRAO)
        s["auto_recal"] = cfg_get("auto_recal", True)
        s["agora"] = time.time()
        return s

    def _atualizar(self, **campos):
        with self.lock:
            self.s.update(campos)
        self.publicar({"t": "estado", **campos})

    # envio de comandos
    def enviar(self, comando):
        with self.serial_lock:
            if not self.ser:
                return False
            if comando != "PING":
                self.diag.append((time.time(), "> " + comando))
            try:
                self.ser.write((comando + "\n").encode())
                return True
            except Exception as e:
                log(f"Erro ao enviar '{comando}': {e}")
                return False

    def sincronizar(self):
        # o Arduino reinicia ao abrir a serial, entao reenvia a configuracao
        time.sleep(0.3)
        self.enviar(f"ALCANCE,{self.s['alcance']}")
        self.enviar(f"PATRULHA,{int(self.s['patrulha'])}")
        self.enviar(f"SOM,{int(self.s['som'])}")
        self.enviar(f"VARRE,{self.s['ang_min']},{self.s['ang_max']}")
        self.enviar(f"MARGEM,{self.s['margem']}")
        if cfg_get("armado", False):
            log("Torre estava armada antes de reiniciar: rearmando")
            self.enviar("ARMAR")

    # mensagens vindas do Arduino
    def processar(self, linha):
        self.diag.append((time.time(), linha))
        partes = linha.split(",")
        tipo = partes[0]
        try:
            if tipo == "R":
                ang, dist = int(partes[1]), int(partes[2])
                if 0 <= ang <= 180:
                    with self.lock:
                        self.radar[ang] = (dist, time.time())
                        self.s["angulo"], self.s["dist"] = ang, dist
                    self.publicar({"t": "r", "ang": ang, "dist": dist})

            elif tipo == "TRK":
                ang, dist, vel = int(partes[1]), int(partes[2]), int(partes[3])
                larg = int(partes[4]) if len(partes) > 4 else 10
                antes = self.s.get("alvo") or {}
                susp = antes.get("suspeito", False)
                self._atualizar(alvo={"ang": ang, "dist": dist, "vel": vel, "larg": larg, "suspeito": susp,
                                      "confirmado": antes.get("confirmado", False)})
                if not susp:
                    self.incidentes.rastreio(ang, dist, vel)

            elif tipo == "EST":
                estado, armado = partes[1], partes[2] == "1"
                antes = self.s["armado"]
                campos = {"estado": estado, "armado": armado}
                if estado != "TRAVADO":
                    campos["alvo"] = None
                if estado != "MANUAL":
                    campos["laser"] = False
                if estado == "CALIBRANDO":
                    self.fundo_novo = {}
                self._atualizar(**campos)
                if armado != antes:
                    cfg_set("armado", armado)
                    self.registrar("armado" if armado else "desarmado", "Armada" if armado else "Desarmada")
                    if not armado:
                        self.incidentes.desarmou()
                        self._atualizar(sugestao=None)

            elif tipo == "SUSP":
                ang, dist = int(partes[1]), int(partes[2])
                self._atualizar(alvo={"ang": ang, "dist": dist, "vel": 0, "suspeito": True})

            elif tipo == "ALVO":
                ang, dist = int(partes[1]), int(partes[2])
                self._atualizar(alvo={"ang": ang, "dist": dist, "vel": 0})
                self.registrar("alvo", f"Alvo travado a {dist} cm", ang, dist)
                self.incidentes.alvo(ang, dist)

            elif tipo == "NIVEL":
                n = int(partes[1])
                self._atualizar(nivel=n)
                if n >= 2 and self.s["armado"] and self.incidentes.sirene():
                    self.registrar("alerta", "Alvo muito perto ou vindo rápido: sirene ligada")

            elif tipo == "IGNORADO":
                ang, dist = int(partes[1]), int(partes[2])
                fundo = list(self.s["fundo"] or [None] * 181)
                for a in range(max(0, ang - 4), min(180, ang + 4) + 1):
                    if fundo[a] is None or fundo[a] < 0 or fundo[a] > dist:
                        fundo[a] = dist
                cfg_set("fundo", fundo)
                self._atualizar(fundo=fundo, alvo=None)
                self.incidentes.ignorado()
                self.registrar("ignorado", f"Objeto parado a {dist} cm ({ang}°): entrou no mapa", ang, dist)

            elif tipo == "CONF":
                alvo = self.s.get("alvo") or {}
                self._atualizar(alvo={**alvo, "confirmado": True} if alvo else None)
                self.registrar("confirmado", "PIR confirmou movimento no alvo",
                               alvo.get("ang"), alvo.get("dist"))
                self.incidentes.confirmado(alvo.get("ang"), alvo.get("dist"))

            elif tipo == "MAPA":
                ang, dist = int(partes[1]), int(partes[2])
                if 0 <= ang <= 180:
                    fundo = list(self.s["fundo"] or [None] * 181)
                    fundo[ang] = dist
                    cfg_set("fundo", fundo)
                    self._atualizar(fundo=fundo)

            elif tipo == "INSTAVEL":
                ang, dist = int(partes[1]), int(partes[2])
                self.incidentes.ignorado()
                self.registrar("instavel", f"Eco falso perto de {ang}°: leituras parecidas ali passam a ser "
                                           f"ignoradas", ang, dist)

            elif tipo == "PERDIDO":
                self._atualizar(alvo=None)
                self.publicar({"t": "perdido"})

            elif tipo == "PIR":
                pir = partes[1] == "1"
                self._atualizar(pir=pir)
                self.incidentes.pir(pir)
                if pir and self.s["armado"]:
                    self.registrar("pir", "Movimento detectado pelo PIR")

            elif tipo == "BASE":
                self.fundo_novo[int(partes[1])] = int(partes[2])

            elif tipo == "BASE_OK":
                fundo = [None] * 181
                for a, d in self.fundo_novo.items():
                    if 0 <= a <= 180:
                        fundo[a] = d
                ts = agora_iso()
                cfg_set("fundo", fundo)
                cfg_set("fundo_ts", ts)
                self._atualizar(fundo=fundo, fundo_ts=ts, calibrado=True, sugestao=None)
                self.registrar("calibracao", f"Ambiente mapeado ({partes[1]} ângulos com referência)")

            elif tipo == "CFG":
                campos = {"alcance": int(partes[1]), "patrulha": partes[2] == "1", "som": partes[3] == "1"}
                if len(partes) >= 6:
                    campos.update(ang_min=int(partes[4]), ang_max=int(partes[5]))
                if len(partes) >= 7:
                    campos["margem"] = int(partes[6])
                self._atualizar(**campos)

            elif tipo == "PONG":
                campos = {"uptime": int(partes[1]), "ram": int(partes[2])}
                if len(partes) > 3:
                    campos["calibrado"] = partes[3] == "1"
                if len(partes) > 5:
                    campos.update(sem_eco=int(partes[4]), pir_bruto=partes[5] == "1")
                self._atualizar(**campos)

            elif tipo == "OI":
                versao = partes[2] if len(partes) > 2 else "?"
                # um OI fora da abertura da porta significa que o Arduino reiniciou sozinho
                sozinho = self.s["conectado"] and time.time() - self.t_porta > 8
                self._atualizar(conectado=True, versao=versao, calibrado=False, nivel=0, alvo=None,
                                reinicios=self.s["reinicios"] + (1 if sozinho else 0))
                if sozinho:
                    self.registrar("aviso", "O Arduino reiniciou sozinho. Costuma ser falta de energia quando o "
                                            "servo se mexe: use uma fonte melhor ou outra porta USB.")
                else:
                    self.registrar("conexao", f"Arduino conectado (firmware {versao})")
                threading.Thread(target=self.sincronizar, daemon=True).start()

            elif tipo == "MSG":
                texto = linha[4:]
                log(f"Arduino: {texto}")
                self.registrar("msg", texto)
        except (IndexError, ValueError):
            log(f"Linha inválida do Arduino: {linha!r}")

    # conexão serial com watchdog
    def iniciar(self):
        threading.Thread(target=self._laco_serial, daemon=True).start()
        threading.Thread(target=self._laco_ping, daemon=True).start()
        threading.Thread(target=self._laco_agenda, daemon=True).start()

    def _abrir(self):
        if self.simular:
            from simulador import ArduinoSimulado
            return ArduinoSimulado(), "simulador"
        import serial  # pyserial
        porta = os.environ.get("PORTA_SERIAL") or next(
            iter(sorted(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*"))), None)
        if not porta:
            return None, None
        return serial.Serial(porta, 115200, timeout=1), porta

    def _laco_ping(self):
        while True:
            time.sleep(5)
            if self.ser:
                self.enviar("PING")

    def _laco_serial(self):
        aviso = False
        while True:
            try:
                ser, porta = self._abrir()
                if not ser:
                    if not aviso:
                        log("Arduino não encontrado na USB. Tentando de novo a cada 3 s...")
                        aviso = True
                    time.sleep(3)
                    continue
                aviso = False
                log(f"Porta aberta: {porta}")
                self.ultimo_rx = self.t_porta = time.time()
                with self.serial_lock:
                    self.ser = ser
                self._atualizar(porta=porta)
                while True:
                    bruto = ser.readline()
                    if bruto:
                        self.ultimo_rx = time.time()
                        linha = bruto.decode(errors="ignore").strip()
                        if linha:
                            self.processar(linha)
                    elif time.time() - self.ultimo_rx > SEM_RESPOSTA_S:
                        raise TimeoutError(f"sem resposta há {SEM_RESPOSTA_S} s, reiniciando o Arduino")
            except Exception as e:
                log(f"Conexão com o Arduino caiu: {e}")
            with self.serial_lock:
                try:
                    if self.ser:
                        self.ser.close()
                except Exception:
                    pass
                self.ser = None
            self._atualizar(conectado=False, estado="DESCONECTADO", nivel=0, alvo=None)
            self.registrar("conexao", "Arduino desconectado")
            time.sleep(3)

    # agenda e manutenção
    @staticmethod
    def dentro_da_agenda(ag, agora):
        if not ag.get("ativa"):
            return None
        hm = agora.strftime("%H:%M")
        dia, ontem = agora.weekday(), (agora.weekday() - 1) % 7
        a, d, dias = ag["armar"], ag["desarmar"], set(ag.get("dias", []))
        if a < d:
            return dia in dias and a <= hm < d
        return (dia in dias and hm >= a) or (ontem in dias and hm < d)

    def _laco_agenda(self):
        anterior = None
        ultima_limpeza = 0
        while True:
            try:
                ag = cfg_get("agenda", AGENDA_PADRAO)
                dentro = self.dentro_da_agenda(ag, datetime.now())
                if dentro is not None and anterior is not None and dentro != anterior:
                    cmd = "ARMAR" if dentro else "DESARMAR"
                    if self.enviar(cmd):
                        self.registrar("agenda", f"Agenda: {'armando' if dentro else 'desarmando'} "
                                                 f"({ag['armar'] if dentro else ag['desarmar']})")
                anterior = dentro
                if time.time() - ultima_limpeza > 86400:   # uma vez por dia
                    ultima_limpeza = time.time()
                    with db() as c:
                        c.execute("DELETE FROM eventos WHERE ts < ?",
                                  ((datetime.now() - timedelta(days=90)).isoformat(),))
                        c.execute("DELETE FROM incidentes WHERE inicio < ?",
                                  ((datetime.now() - timedelta(days=365)).isoformat(),))
            except Exception as e:
                log(f"Agenda: erro {e}")
            time.sleep(20)


# Web
app = Flask(__name__, static_folder=os.path.join(BASE, "static"))
torre = None

COMANDOS = {
    "ARMAR": None, "DESARMAR": None, "CALIBRAR": None,
    "MODO": ("AUTO", "MANUAL"),
    "MIRA": (0, 180), "LASER": (0, 1), "ALCANCE": (10, 300),
    "PATRULHA": (0, 1), "SOM": (0, 1), "MARGEM": (5, 80),
}


@app.before_request
def exigir_senha():
    senha = os.environ.get("SENHA_PAINEL", "")
    if not senha:
        return None
    auth = request.authorization
    if auth and auth.password == senha:
        return None
    return Response("Senha necessária", 401, {"WWW-Authenticate": 'Basic realm="Torre Sentinela"'})


@app.get("/")
def index():
    return send_from_directory(app.static_folder, "index.html")


@app.get("/api/estado")
def api_estado():
    return jsonify(torre.snapshot())


@app.get("/api/stream")
def api_stream():
    q = torre.inscrever()

    def gerar():
        try:
            yield f"data: {json.dumps({'t': 'snapshot', **torre.snapshot()})}\n\n"
            while True:
                try:
                    ev = q.get(timeout=15)
                    yield f"data: {json.dumps(ev)}\n\n"
                except queue.Empty:
                    yield ": ping\n\n"
        finally:
            torre.desinscrever(q)

    return Response(gerar(), mimetype="text/event-stream",
                    headers={"Cache-Control": "no-cache", "X-Accel-Buffering": "no"})


@app.post("/api/comando")
def api_comando():
    dados = request.get_json(silent=True) or {}
    cmd = str(dados.get("cmd", "")).upper()
    if cmd == "VARRE":
        try:
            mn = max(0, min(170, int(dados.get("min"))))
            mx = max(mn + 10, min(180, int(dados.get("max"))))
        except (TypeError, ValueError):
            return jsonify(erro="informe min e max"), 400
        cfg_set("ang_min", mn)
        cfg_set("ang_max", mx)
        linha = f"VARRE,{mn},{mx}"
    elif cmd in COMANDOS:
        regra = COMANDOS[cmd]
        linha = cmd
        if regra is not None:
            valor = dados.get("valor")
            if isinstance(regra[0], str):
                valor = str(valor).upper()
                if valor not in regra:
                    return jsonify(erro="valor inválido"), 400
            else:
                try:
                    valor = int(valor)
                except (TypeError, ValueError):
                    return jsonify(erro="valor inválido"), 400
                valor = max(regra[0], min(regra[1], valor))
            linha = f"{cmd},{valor}"
            if cmd in ("ALCANCE", "PATRULHA", "SOM", "MARGEM"):
                cfg_set(cmd.lower(), valor if cmd in ("ALCANCE", "MARGEM") else bool(valor))
            if cmd == "LASER":
                torre._atualizar(laser=bool(valor))
    else:
        return jsonify(erro="comando inválido"), 400
    if not torre.enviar(linha):
        return jsonify(erro="Arduino desconectado"), 503
    return jsonify(ok=True)


@app.get("/api/diagnostico")
def api_diagnostico():
    snap = torre.snapshot()
    snap.pop("radar", None)
    linhas = [f"# Torre Sentinela - diagnostico gerado em {agora_iso()}",
              "# estado: " + json.dumps(snap, ensure_ascii=False)]
    with db() as c:
        for r in c.execute("SELECT * FROM incidentes ORDER BY id DESC LIMIT 30"):
            d = dict(r)
            d.pop("trajeto", None)
            linhas.append("# incidente: " + json.dumps(d, ensure_ascii=False))
    inicio = torre.diag[0][0] if torre.diag else time.time()
    for t, l in list(torre.diag):
        linhas.append(f"{t - inicio:9.2f}s  {l}")
    nome = f"diagnostico-torre-{datetime.now():%Y%m%d-%H%M}.txt"
    return Response("\n".join(linhas) + "\n", mimetype="text/plain",
                    headers={"Content-Disposition": f'attachment; filename="{nome}"'})


@app.get("/api/agenda")
def api_agenda():
    return jsonify(cfg_get("agenda", AGENDA_PADRAO))


@app.post("/api/agenda")
def api_agenda_salvar():
    d = request.get_json(silent=True) or {}
    if not (hhmm_valido(d.get("armar")) and hhmm_valido(d.get("desarmar"))) or d["armar"] == d["desarmar"]:
        return jsonify(erro="use horários no formato HH:MM, diferentes entre si"), 400
    dias = sorted({int(x) for x in d.get("dias", []) if str(x).isdigit() and 0 <= int(x) <= 6})
    ag = {"ativa": bool(d.get("ativa")), "armar": d["armar"], "desarmar": d["desarmar"], "dias": dias}
    cfg_set("agenda", ag)
    torre.publicar({"t": "estado", "agenda": ag})
    torre.registrar("agenda", "Agenda " + (f"ativa: arma {ag['armar']}, desarma {ag['desarmar']}"
                                           if ag["ativa"] else "desligada"))
    return jsonify(ok=True, agenda=ag)


@app.post("/api/preferencias")
def api_preferencias():
    d = request.get_json(silent=True) or {}
    for chave in ("auto_recal",):
        if chave in d:
            cfg_set(chave, bool(d[chave]))
            torre.publicar({"t": "estado", chave: bool(d[chave])})
    return jsonify(ok=True)


@app.get("/api/incidentes")
def api_incidentes():
    limite = max(1, min(200, request.args.get("limite", 30, type=int)))
    with db() as c:
        rows = c.execute("SELECT * FROM incidentes ORDER BY id DESC LIMIT ?", (limite,)).fetchall()
    out = []
    for r in rows:
        d = dict(r)
        d["trajeto"] = json.loads(d["trajeto"] or "[]")
        out.append(d)
    return jsonify(out)


@app.get("/api/estatisticas")
def api_estatisticas():
    dias = max(1, min(30, request.args.get("dias", 7, type=int)))
    desde = (datetime.now() - timedelta(days=dias)).isoformat(timespec="seconds")
    with db() as c:
        rows = c.execute("SELECT inicio, classe FROM incidentes WHERE inicio >= ?", (desde,)).fetchall()
    por_hora = [0] * 24
    por_dia = {}
    for r in rows:
        if r["classe"] == "pessoa":
            por_hora[int(r["inicio"][11:13])] += 1
        dia = r["inicio"][:10]
        por_dia.setdefault(dia, {"pessoa": 0, "falso": 0, "outros": 0})
        chave = r["classe"] if r["classe"] in ("pessoa", "falso") else "outros"
        por_dia[dia][chave] += 1
    lista_dias = []
    for i in range(dias - 1, -1, -1):
        dia = (datetime.now() - timedelta(days=i)).strftime("%Y-%m-%d")
        lista_dias.append({"dia": dia, **por_dia.get(dia, {"pessoa": 0, "falso": 0, "outros": 0})})
    total = len(rows)
    falsos = sum(1 for r in rows if r["classe"] == "falso")
    return jsonify(por_hora=por_hora, por_dia=lista_dias, total=total, falsos=falsos,
                   taxa_falsos=round(100 * falsos / total) if total else 0)


@app.get("/api/eventos")
def api_eventos():
    limite = max(1, min(500, request.args.get("limite", 60, type=int)))
    with db() as c:
        rows = c.execute("SELECT * FROM eventos ORDER BY id DESC LIMIT ?", (limite,)).fetchall()
    return jsonify([dict(r) for r in rows])


@app.get("/api/deteccoes")
def api_deteccoes():
    horas = max(1, min(168, request.args.get("horas", 24, type=int)))
    desde = (datetime.now() - timedelta(hours=horas)).isoformat(timespec="seconds")
    with db() as c:
        rows = c.execute("SELECT ts, ang, dist FROM eventos WHERE tipo='alvo' AND ts >= ? ORDER BY id",
                         (desde,)).fetchall()
    return jsonify([dict(r) for r in rows])


def main():
    global torre
    carregar_config_env()
    ap = argparse.ArgumentParser(description="Servidor da Torre Sentinela")
    ap.add_argument("--simular", action="store_true", help="roda sem Arduino, com dados falsos")
    ap.add_argument("--porta-web", type=int, default=int(os.environ.get("PORTA_WEB", 5000)))
    args = ap.parse_args()

    iniciar_banco()
    torre = Torre(simular=args.simular)
    torre.iniciar()
    log(f"Painel em http://0.0.0.0:{args.porta_web}  (no notebook use o IP do Rasp)")
    if os.environ.get("SENHA_PAINEL"):
        log("Painel protegido por senha (SENHA_PAINEL)")
    app.run(host="0.0.0.0", port=args.porta_web, threaded=True, debug=False)


if __name__ == "__main__":
    main()
