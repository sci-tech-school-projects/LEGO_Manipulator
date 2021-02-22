# https://www.pyimagesearch.com/2018/05/21/an-opencv-barcode-and-qr-code-scanner-with-zbar/
from pyzbar import pyzbar
import cv2
import sys

url = 'QR/start.png'
image = cv2.imread(url)
# cv2.imshow("image", image)
# cv2.waitKey(0)
# cv2.destroyAllWindows()

qrs = pyzbar.decode(image)

for qr in qrs:
    print(qr)
    print(qr[2][0])
    # cv2.rectangle(frame, (x, y), (x + w, y + h),(255,0,0),2)

    cv2.rectangle(image, (qr[2][0], qr[2][1]), (qr[2][0] + qr[2][2], qr[2][1] + qr[2][3]),(255,0,0),2)

    cv2.imshow("image", image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
