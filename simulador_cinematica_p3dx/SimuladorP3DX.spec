# Executável do simulador para Windows (PyInstaller), num arquivo só.
#
# Uso, nesta pasta:
#     python -m PyInstaller SimuladorP3DX.spec
# Sai em dist/SimuladorP3DX.exe. Ele leva o Python, o servidor.py e a página; ao abrir,
# serve a página em localhost e abre o navegador, como o "python servidor.py".
# A janela de console mostra o endereço; fechar a janela encerra o simulador.

pagina = [(f, '.') for f in ('index.html', 'estilo.css', 'cinematica.js', 'interface.js')]

a = Analysis(
    ['servidor.py'],
    datas=pagina,
    # o servidor só usa a biblioteca padrão; o resto não precisa ir junto
    excludes=['tkinter', 'unittest', 'pydoc', 'numpy', 'matplotlib'],
)
pyz = PYZ(a.pure)
exe = EXE(
    pyz,
    a.scripts,
    a.binaries,
    a.datas,
    name='SimuladorP3DX',
    console=True,
    upx=False,
)
