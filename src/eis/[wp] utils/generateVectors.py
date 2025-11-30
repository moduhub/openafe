import re

""" Example of log:
Log 1:
float: 1000.00
6490
-47865
--------------
float: 1269.53125000
5752
-51962
--------------
float: 1586.91406250
10624
-47899
--------------
float: 2001.95312500
9453
-48825
[...]
"""

arquivo = "log"
def parse_arquivo(caminho):
    with open(caminho, "r") as f:
        linhas = [l.strip() for l in f.readlines()]

    testes = {}
    teste_atual = None
    estado = "IGNORE"
    real_temp = None
    imag_temp = None

    freqs = []
    dados = {}

    for linha in linhas:

        if "Log" in linha:
            nome = linha.replace(":", "")
            teste_atual = nome
            dados[teste_atual] = []
            continue

        if linha.startswith("float:"):
            freq = float(linha.split(":")[1].strip())
            freqs.append(freq)
            estado = "REAL"
            continue

        if linha.startswith("--------------"):
            continue

        if estado == "REAL":
            if linha != "":
                real_temp = float(linha)
                estado = "IMAG"
            continue

        elif estado == "IMAG":
            if linha != "":
                imag_temp = float(linha)
                dados[teste_atual].append((real_temp, imag_temp))
                estado = "IGNORE"
            continue

    return freqs, dados


freqs, dados = parse_arquivo(arquivo)

print("// Frequences")
print("freqs = [")
print("    " + ", ".join(str(f) for f in freqs) + " ];\n")

for k_i, nome in enumerate(dados.keys(), start=1):
    print(f"// {nome}")
    print(f"z{k_i} = [")
    linhas = []
    for (real, imag) in dados[nome]:
        sinal = "+" if imag >= 0 else "-"
        linhas.append(f"{real} {sinal} {abs(imag)}*%i")
    print("    " + ",\n    ".join(linhas))
    print("];\n")
