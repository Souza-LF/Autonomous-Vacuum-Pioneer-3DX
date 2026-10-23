"""
Referência para os testes: os dois programas da Cinemática - Parte B (Prof. André
Marcato), copiados dos slides 27 e 29 (modelo diferencial) e 48 e 50 (modelo
incremental), sem a parte gráfica.

Grava referencia.json com as entradas de cada programa e a trajetória completa.
O teste_cinematica.js compara essa trajetória, ponto a ponto, com cinematica.js.

Uso (na pasta do simulador):  python testes/referencia_professor.py
"""
import json
from pathlib import Path

import numpy as np


def modelo_diferencial():
    # slide 27: pose inicial e parâmetros
    P = np.array([0.0, 0.0, 0.0])
    r = 0.5 * (195.0 / 1000.0)
    l = 0.5 * (381.0 / 1000.0)
    wl = np.deg2rad(60.0)
    wr = np.deg2rad(70.0)
    dt = 0.1
    tmax = 30.0
    R = P.reshape(3, 1)

    # slide 29: integração da pose
    for t in np.arange(0.0, tmax + dt, dt):
        th = P[2]
        v = r / 2.0 * (wr + wl)
        w = r / (2.0 * l) * (wr - wl)
        dP = np.array([v * np.cos(th), v * np.sin(th), w])
        P = P + dP * dt
        R = np.column_stack((R, P))

    return dict(r=r, l=l, wl=wl, wr=wr, dt=dt, tmax=tmax, passos=R.shape[1] - 1, R=R.T.tolist())


def modelo_incremental():
    # slide 48: pose inicial e parâmetros
    P = np.array([0.0, 0.0, np.deg2rad(0.0)])
    r = 0.5 * (195.0 / 1000.0)
    l = 0.5 * (381.0 / 1000.0)
    wl = np.deg2rad(50.0)
    wr = np.deg2rad(80.0)
    dt = 0.2
    tmax = 30.0
    R = P.reshape(3, 1)

    # slide 50: atualização incremental da pose
    for t in np.arange(0.0, tmax + dt, dt):
        v = r / 2.0 * (wr + wl)
        w = r / (2.0 * l) * (wr - wl)
        th = P[2]
        delta_s = v * dt
        delta_th = w * dt
        delta_p = np.array([
            delta_s * np.cos(th + delta_th / 2.0),
            delta_s * np.sin(th + delta_th / 2.0),
            delta_th,
        ])
        P = P + delta_p
        R = np.column_stack((R, P))

    return dict(r=r, l=l, wl=wl, wr=wr, dt=dt, tmax=tmax, passos=R.shape[1] - 1, R=R.T.tolist())


if __name__ == "__main__":
    ref = {"diferencial": modelo_diferencial(), "incremental": modelo_incremental()}
    saida = Path(__file__).with_name("referencia.json")
    saida.write_text(json.dumps(ref, indent=1), encoding="utf-8")
    for nome, d in ref.items():
        x, y, th = d["R"][-1]
        print(f"{nome:12s} passos={d['passos']}  pose final = ({x:.6f} m, {y:.6f} m, {np.rad2deg(th):.4f} graus)")
    print("gravado em", saida)
