import csv

import matplotlib.pyplot as plt
import serial


PUERTO = "COM3"
BAUDIOS = 9600


def guardar_grafica(tiempos, setpoints, iluminacion, posiciones, pwm):
    if not tiempos:
        print("No hay datos para graficar.")
        return

    plt.figure(figsize=(10, 6))
    plt.plot(tiempos, setpoints, label="Setpoint")
    plt.plot(tiempos, iluminacion, label="Iluminación LDR")
    plt.plot(tiempos, posiciones, label="Posición")
    plt.plot(tiempos, pwm, label="PWM")
    plt.xlabel("Tiempo (s)")
    plt.ylabel("Valor")
    plt.title("Control de iluminación - Problema B")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig("grafica_iluminacion.png", dpi=150)
    plt.show()


def main():
    tiempos = []
    setpoints = []
    iluminacion = []
    posiciones = []
    pwm = []

    nombre_csv = "registro_iluminacion.csv"

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
            "tiempo_ms",
            "setpoint",
            "ldr",
            "posicion",
            "pwm",
            "sentido"
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

                if len(partes) != 7:
                    continue

                try:
                    tiempo_ms = int(partes[1])
                    setpoint = int(partes[2])
                    ldr = int(partes[3])
                    posicion = int(partes[4])
                    pwm_actual = int(partes[5])
                    sentido = partes[6]
                except ValueError:
                    continue

                escritor.writerow([
                    tiempo_ms,
                    setpoint,
                    ldr,
                    posicion,
                    pwm_actual,
                    sentido
                ])
                archivo.flush()

                tiempos.append(tiempo_ms / 1000.0)
                setpoints.append(setpoint)
                iluminacion.append(ldr)
                posiciones.append(posicion)
                pwm.append(pwm_actual)

        except KeyboardInterrupt:
            print("\nRegistro finalizado.")

        finally:
            puerto.close()

    print(f"Datos guardados en {nombre_csv}")

    if tiempos:
        errores = [abs(sp - luz) for sp, luz in zip(setpoints, iluminacion)]
        error_medio = sum(errores) / len(errores)
        print(f"Error medio absoluto setpoint-LDR: {error_medio:.2f} unidades ADC")

    guardar_grafica(
        tiempos,
        setpoints,
        iluminacion,
        posiciones,
        pwm
    )


if __name__ == "__main__":
    main()
