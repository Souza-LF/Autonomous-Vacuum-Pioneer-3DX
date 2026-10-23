"""
Servidor local do simulador: serve esta pasta em http://localhost:8000 e abre o navegador.

Uso (de qualquer pasta):
    python servidor.py                  porta 8000, ou a próxima livre
    python servidor.py 8080             outra porta
    python servidor.py --sem-navegador  só o servidor
Ctrl+C encerra.

Só usa a biblioteca padrão. Escuta apenas em 127.0.0.1, então não fica visível na rede.
"""
import functools
import http.server
import sys
import threading
import webbrowser
from pathlib import Path

PASTA = Path(__file__).resolve().parent


class Servidor(http.server.ThreadingHTTPServer):
    # no Windows, SO_REUSEADDR deixaria abrir uma porta que outro programa já usa
    allow_reuse_address = sys.platform != "win32"


class Arquivos(http.server.SimpleHTTPRequestHandler):
    def end_headers(self):
        # sem cache: depois de editar o código, basta recarregar a página
        self.send_header("Cache-Control", "no-store")
        super().end_headers()

    def log_message(self, formato, *args):
        pass


def abrir(porta_inicial):
    handler = functools.partial(Arquivos, directory=str(PASTA))
    for porta in range(porta_inicial, porta_inicial + 20):
        try:
            return Servidor(("127.0.0.1", porta), handler), porta
        except OSError:
            continue
    raise SystemExit(f"nenhuma porta livre entre {porta_inicial} e {porta_inicial + 19}")


def main():
    args = sys.argv[1:]
    portas = [int(a) for a in args if a.isdigit()]
    servidor, porta = abrir(portas[0] if portas else 8000)
    url = f"http://localhost:{porta}/"
    print(f"Simulador do P3-DX em {url}")
    print("Ctrl+C para encerrar.")
    if "--sem-navegador" not in args:
        threading.Timer(0.5, webbrowser.open, args=(url,)).start()
    with servidor:
        try:
            servidor.serve_forever()
        except KeyboardInterrupt:
            print("\nencerrado")


if __name__ == "__main__":
    main()
