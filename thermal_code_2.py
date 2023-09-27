import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import busio
import board
import adafruit_mlx90640

# Inicialize o barramento I2C
i2c = busio.I2C(board.SCL, board.SDA, frequency=800000)

# Inicialize a câmera térmica
mlx = adafruit_mlx90640.MLX90640(i2c, address=0x33)
mlx.refresh_rate = adafruit_mlx90640.RefreshRate.REFRESH_2_HZ

# Capture uma imagem
frame = [0] * 768
mlx.getFrame(frame)

# Plote a imagem
fig, ax = plt.subplots()
im = ax.imshow(np.reshape(frame, (24, 32)), cmap='hot', vmin=20, vmax=180)
fig.colorbar(im)

# Encontre o valor mínimo e máximo
valor_minimo = min(frame)
valor_maximo = max(frame)

print("Valor mínimo:", valor_minimo)
print("Valor máximo:", valor_maximo)

# Salvar a figura em um arquivo de imagem
fig.savefig('imagem.png')
