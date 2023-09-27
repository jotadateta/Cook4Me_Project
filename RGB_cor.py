import cv2

image = cv2.imread('reference_image.jpg')

b = image.copy()
#extract red channel
blue = b[:,:,0]


g = image.copy()
#extract red channel
green = g[:,:,1]

r = image.copy()
#extract red channel
red = r[:,:,2]


# RGB - Blue
cv2.imshow('B-RGB', blue)

# RGB - Green
cv2.imshow('G-RGB', green)

# RGB - Red
cv2.imshow('R-RGB', red)

cv2.imshow('Real', image)

cv2.waitKey(0)
cv2.destroyAllWindows()
