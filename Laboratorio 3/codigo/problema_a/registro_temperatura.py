import csv
from datetime import datetime

import matplotlib.pyplot as plt
import serial


PUERTO = "COM3"
BAUDIOS = 9600


def guardar_grafica(tiempos, temperaturas, calefactor, pwm, ideal_min, ideal_max):
    if not tiempos:
        print("No hay datos para graficar.")
        return

    fig, (ax_temp, ax_accion) = plt.subplots(2, 1, figsize=(10, 7), sharex=True)

    ax_temp.plot(tiempos, temperaturas, marker="o", label="Temperatura")
    ax_temp.plot(tiempos, ideal_min, linestyle="--", label="Ideal mínimo")
    ax_temp.plot(tiempos, ideal_max, linestyle="--", label="Ideal máximo")
    ax_temp.fill_between(tiempos, ideal_min, ideal_max, alpha=0.15)
    ax_temp.set_ylabel("Temperatura (°C)")
    ax_temp.set_title("Control de temperatura - Problema A")
    ax_temp.grid(True)
    ax_temp.legend()

    ax_accion.step(tiempos, calefactor, where="post", label="Calefactor ON/OFF")
    ax_accion.step(tiempos, pwm, where="post", label="PWM ventilador")
    ax_accion.set_xlabel("Tiempo (s)")
    ax_accion.set_ylabel("Acción")
    ax_accion.set_yticks([0, 1, 85, 170, 255])
    ax_accion.grid(True)
    ax_accion.legend()

    fig.tight_layout()
    fig.savefig("grafica_temperatura.png", dpi=150)
    plt.show()


def main():
    tiempos = []
    temperaturas = []
    calefactor = []
    pwm = []
    ideal_min = []
    ideal_max = []

    nombre_csv = "registro_temperatura.csv"

    try:
        puerto = serial.Serial(PUERTO, BAUDIOS, timeout=1)
    except serial.SerialException as error:
        print(f"No se pudo abrir {PUERTO}: {error}")
        return

    print(f"Escuchando {PUERTO} a {BAUDIOS} baud.")
    print("Presioná Ctrl+C para terminar, guardar y graficar.\n")

    with open(nombre_csv, "w", newline="", encoding="utf-8") as archivo:
        escritor = csv.writer(archivo)
        escritor.writerow([
            "tiempo_s",
            "temperatura_C",
            "calefactor",
            "pwm",
            "ideal_min",
            "ideal_max",
            "estado"
        ])

        try:
            while True:
                linea = puerto.readline().decode("utf-8", errors="ignore").strip()

                if not linea:
                    continue

                print(linea)

                if not linea.startswith("DATA,"):
                    continue

                partes = linea.split(",")

                if len(partes) != 8:
                    continue

                try:
                    tiempo = int(partes[1])
                    temperatura = int(partes[2])
                    calefactor_actual = int(partes[3])
                    pwm_actual = int(partes[4])
                    minimo = int(partes[5])
                    maximo = int(partes[6])
                    estado = partes[7]
                except ValueError:
                    continue

                escritor.writerow([
                    tiempo,
                    temperatura,
                    calefactor_actual,
                    pwm_actual,
                    minimo,
                    maximo,
                    estado
                ])
                archivo.flush()

                tiempos.append(tiempo)
                temperaturas.append(temperatura)
                calefactor.append(calefactor_actual)
                pwm.append(pwm_actual)
                ideal_min.append(minimo)
                ideal_max.append(maximo)

        except KeyboardInterrupt:
            print("\nRegistro finalizado.")

        finally:
            puerto.close()

    print(f"Datos guardados en {nombre_csv}")
    guardar_grafica(
        tiempos,
        temperaturas,
        calefactor,
        pwm,
        ideal_min,
        ideal_max
    )


if __name__ == "__main__":
    main()
