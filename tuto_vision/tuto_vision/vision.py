import cv2
import numpy as np
from skimage.measure import label, regionprops
import matplotlib.pyplot as plt
import math

def screen() :
    # connect to a sensor (0: webcam)
    cap=cv2.VideoCapture(0)

    # capture an image
    ret, frame=cap.read()

    # Select ROI
    r = cv2.selectROI(frame)

    # Crop image
    imCrop = frame[int(r[1]):int(r[1]+r[3]), int(r[0]):int(r[0]+r[2])]

    average_h = np.mean(imCrop[:,:,0])
    average_s = np.mean(imCrop[:,:,1])
    average_v = np.mean(imCrop[:,:,2])

    print(average_h,average_s,average_v)

    # Display cropped image
    cv2.imshow("Image", imCrop)
    cv2.waitKey(0)

def souris(event, x, y, flags, param):
    global lo, hi, color, hsv_px

    if event == cv2.EVENT_MOUSEMOVE:
        # Conversion des trois couleurs RGB sous la souris en HSV
        px = frame[y,x]
        px_array = np.uint8([[px]])
        hsv_px = cv2.cvtColor(px_array,cv2.COLOR_BGR2HSV)

    if event==cv2.EVENT_MBUTTONDBLCLK:
        color=image[y, x][0]

    if event==cv2.EVENT_LBUTTONDOWN:
        if color>5:
            color-=1

    if event==cv2.EVENT_RBUTTONDOWN:
        if color<250:
            color+=1

    lo[0]=color-10
    hi[0]=color+10

    color=100

    lo=np.array([color-5, 100, 50])
    hi=np.array([color+5, 255,255])

    color_info=(0, 0, 255)

    cap=cv2.VideoCapture(0)
    cv2.namedWindow('Camera')
    cv2.setMouseCallback('Camera', souris)
    hsv_px = [0,0,0]

    # Creating morphological kernel
    kernel = np.ones((3, 3), np.uint8)

    while True:
        ret, frame=cap.read()
        image=cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask=cv2.inRange(image, lo, hi)

        # Flouttage de l'image
        image=cv2.blur(image, (7, 7))
        # Erosion d'un mask
        mask=cv2.erode(mask, None, iterations=4)
        # dilatation d'un mask
        mask=cv2.dilate(mask, None, iterations=4)
        image2=cv2.bitwise_and(frame, frame, mask = mask)
        cv2.putText(frame, "Couleur: {:d}".format(color), (10, 30), cv2.FONT_HERSHEY_DUPLEX, 1, color_info, 1, cv2.LINE_AA)

        # Affichage des composantes HSV sous la souris sur l'image
        pixel_hsv = " ".join(str(values) for values in hsv_px)
        font = cv2.FONT_HERSHEY_SIMPLEX
        elements=cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        if len(elements) > 0:
            c=max(elements, key=cv2.contourArea)
            ((x, y), rayon)=cv2.minEnclosingCircle(c)
            if rayon>30:
                cv2.circle(image2, (int(x), int(y)), int(rayon), color_info, 2)
                cv2.circle(frame, (int(x), int(y)), 5, color_info, 10)
                cv2.line(frame, (int(x), int(y)), (int(x)+150, int(y)), color_info, 2)
                cv2.putText(frame, "Objet !!!", (int(x)+10, int(y) -10), cv2.FONT_HERSHEY_DUPLEX, 1, color_info, 1, cv2.LINE_AA)
        cv2.putText(frame, "px HSV: "+pixel_hsv, (10, 260),
                font, 1, (255, 255, 255), 1, cv2.LINE_AA)

        cv2.imshow('Camera', frame)
        cv2.imshow('image2', image2)
        cv2.imshow('Mask', mask)

        if cv2.waitKey(1)&0xFF==ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()

def skimage() :

    image = cv2.imread('/home/bot/nukacola.png')

    # passage en niveau de gris
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    ###### extration des régions avec la lib skimage
    
    # Binarisation de l'image
    ret, thresh = cv2.threshold(gray, 127, 255, 1)
    cv2.imshow("image seuillée",thresh)
    cv2.waitKey(0)
    
    # extraction des régions et des propriétés des régions
    label_img = label(thresh)
    regions = regionprops(label_img)
    print(regions)
    cv2.waitKey(0)
    
    # affichage des régions et des boites englobantes
    fig, ax = plt.subplots()
    ax.imshow(thresh, cmap=plt.cm.gray)
    
    for props in regions:
        y0, x0 = props.centroid
        orientation = props.orientation
        x1 = x0 + math.cos(orientation) * 0.5 * props.minor_axis_length
        y1 = y0 - math.sin(orientation) * 0.5 * props.minor_axis_length
        x2 = x0 - math.sin(orientation) * 0.5 * props.major_axis_length
        y2 = y0 - math.cos(orientation) * 0.5 * props.major_axis_length
    
        ax.plot((x0, x1), (y0, y1), '-r', linewidth=2.5)
        ax.plot((x0, x2), (y0, y2), '-r', linewidth=2.5)
        ax.plot(x0, y0, '.g', markersize=15)
    
        minr, minc, maxr, maxc = props.bbox
        bx = (minc, maxc, maxc, minc, minc)
        by = (minr, minr, maxr, maxr, minr)
        ax.plot(bx, by, '-b', linewidth=2.5)
    
    ax.axis((0, 600, 600, 0))
    plt.show()
    
    cv2.waitKey(0)

def template_matching() :
    # charger l'image dans laquelle on cherche l'objet
    img_rgb = cv2.imread('/home/bot/car.png')
    img_gray = cv2.cvtColor(img_rgb, cv2.COLOR_BGR2GRAY)
    
    
    # charger le template de l'objet à rechercher
    template = cv2.imread('/home/bot/template.png',0)
    
    # Récupération des dimensions de l'image
    w, h = template.shape[::-1]
    
    # Application du template atching
    res = cv2.matchTemplate(img_gray,template,cv2.TM_CCOEFF_NORMED)
    
    # Sélection des meilleurs matched objects
    threshold = 0.8
    loc = np.where( res >= threshold)
    
    # Affichage de la boite englobante de chaque objet détecté
    for pt in zip(*loc[::-1]):
        cv2.rectangle(img_rgb, pt, (pt[0] + w, pt[1] + h), (0,0,255), 2)
    
    cv2.imshow('Fenetre', img_rgb)
    cv2.waitKey(0)

