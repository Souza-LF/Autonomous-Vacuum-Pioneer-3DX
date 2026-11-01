# Executável da simulação de controle para Windows (PyInstaller), num arquivo só.
#
# Uso, nesta pasta:
#     python -m PyInstaller ControleP3DX.spec
# Sai em dist/ControleP3DX.exe. Ele leva o Python, o servidor.py e as duas páginas; ao abrir,
# serve a página de escolha em localhost e abre o navegador, como o "python servidor.py".
# A janela de console mostra o endereço; fechar a janela encerra a simulação.

paginas = [(f, '.') for f in ('index.html', 'simulacao.html', 'estilo.css', 'cinematica.js',
                              'controladores.js', 'escolha.js', 'simulacao.js')]

a = Analysis(
    ['servidor.py'],
    datas=paginas,
    # o servidor só usa a biblioteca padrão; o resto não precisa ir junto
    excludes=['tkinter', 'unittest', 'pydoc', 'numpy', 'matplotlib'],
)
pyz = PYZ(a.pure)
exe = EXE(
    pyz,
    a.scripts,
    a.binaries,
    a.datas,
    name='ControleP3DX',
    console=True,
    upx=False,
)
