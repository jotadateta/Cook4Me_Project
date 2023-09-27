import cv2
import numpy as np

# Carregar imagem
img = cv2.imread('../images/frame_teste.jpg')
# Fazer uma cópia da imagem usando numpy
img_copy = np.copy(img)

method = "houghcircles"

# Converter a imagem para tons de cinza
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

# Aplicar filtro Gaussiano para reduzir ruído

img_blur = cv2.GaussianBlur(gray, (5, 5), 0)

# Detectar círculos usando a transformada de Hough
circles = cv2.HoughCircles(img_blur, cv2.HOUGH_GRADIENT, 1, 30,
                        param1=100, param2=100, minRadius=20, maxRadius=0)

# Criar máscara circular com o mesmo tamanho do círculo detectado
mask = np.zeros_like(gray)

# Desenhar círculos detectados na imagem original
if circles is not None:
    circles = np.round(circles[0, :]).astype("int")
    for (x, y, r) in circles:
        cv2.circle(img, (x, y), r+10, (0, 255, 0), 2)
        cv2.circle(img, (x, y), 2, (0, 0, 255), 3)
        cv2.circle(mask, (x, y), r+10, 255, -1)

# Obter as coordenadas delimitadoras da máscara

y, x = np.where(mask == 255)
if y.size > 0 and x.size > 0:
    top, left = np.min(y), np.min(x)
    bottom, right = np.max(y), np.max(x)
    h, w = bottom - top + 1, right - left + 1
    print("x"+str(top))
    print("y"+str(left))
    print("w"+str(w))
    print("h"+str(h))

    # Aplicar a máscara na imagem original para extrair a região de interesse
    result = cv2.bitwise_and(img, img, mask=mask)

        
    # Separa os canais de cor
    b, g, r = cv2.split(result)

    # Aplica a equalização adaptativa de contraste em cada canal de cor
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
    b_cl = clahe.apply(b)
    g_cl = clahe.apply(g)
    r_cl = clahe.apply(r)

    # Combina os canais de volta para formar a imagem final
    cl_img = cv2.merge((b_cl, g_cl, r_cl))

    roi = cl_img[top:top+h, left:left+w]
else:
    print("Não foram encontrados pixels brancos na máscara.")


# Exibir imagem com círculos detectados
cv2.imshow("Image", img)
cv2.imshow("Image_original", img_copy)
cv2.imshow('Imagem ROi', result)
cv2.imshow('Imagem final', roi)
cv2.waitKey(0)
cv2.destroyAllWindows()




