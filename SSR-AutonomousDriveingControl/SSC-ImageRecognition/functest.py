

import numpy as np
from matplotlib import pyplot as plt
import matplotlib.pyplot as plt

import numpy as np
import cv2#opencv-contrib-python not opencv-python
import numpy as np
import cv2
import numpy as np

from skimage.morphology import skeletonize
from OutputController import OutputController

# Zhang-Suenのアルゴリズムを用いて2値化画像を細線化します
def Zhang_Suen_thinning(binary_image):
    # オリジナルの画像をコピー
    image_thinned = binary_image.copy()
    # 初期化します。この値は次のwhile文の中で除かれます。
    changing_1 = changing_2 = [1]
    while changing_1 or changing_2:
        # ステップ1
        changing_1 = []
        rows, columns = image_thinned.shape
        for x in range(1, rows - 1):
            for y in range(1, columns -1):
                p2, p3, p4, p5, p6, p7, p8, p9 = neighbour_points = neighbours(x, y, image_thinned)
                if (image_thinned[x][y] == 1 and
                    2 <= sum(neighbour_points) <= 6 and # 条件2
                    count_transition(neighbour_points) == 1 and # 条件3
                    p2 * p4 * p6 == 0 and # 条件4
                    p4 * p6 * p8 == 0): # 条件5
                    changing_1.append((x,y))
        for x, y in changing_1:
            image_thinned[x][y] = 0
        # ステップ2
        changing_2 = []
        for x in range(1, rows - 1):
            for y in range(1, columns -1):
                p2, p3, p4, p5, p6, p7, p8, p9 = neighbour_points = neighbours(x, y, image_thinned)
                if (image_thinned[x][y] == 1 and
                    2 <= sum(neighbour_points) <= 6 and # 条件2
                    count_transition(neighbour_points) == 1 and # 条件3
                    p2 * p4 * p8 == 0 and # 条件4
                    p2 * p6 * p8 == 0): # 条件5
                    changing_2.append((x,y))
        for x, y in changing_2:
            image_thinned[x][y] = 0        

    return image_thinned

# 2値画像の黒を1、白を0とするように変換するメソッドです
def black_one(binary):
    bool_image = binary.astype(bool)
    inv_bool_image = ~bool_image
    return inv_bool_image.astype(int)

# 画像の外周を0で埋めるメソッドです
def padding_zeros(image):
    import numpy as np
    m,n = np.shape(image)
    padded_image = np.zeros((m+2,n+2))
    padded_image[1:-1,1:-1] = image
    return padded_image

# 外周1行1列を除くメソッドです。
def unpadding(image):
    return image[1:-1, 1:-1]

# 指定されたピクセルの周囲のピクセルを取得するメソッドです
def neighbours(x, y, image):
    return [image[x-1][y], image[x-1][y+1], image[x][y+1], image[x+1][y+1], # 2, 3, 4, 5
             image[x+1][y], image[x+1][y-1], image[x][y-1], image[x-1][y-1]] # 6, 7, 8, 9

# 0→1の変化の回数を数えるメソッドです
def count_transition(neighbours):
    neighbours += neighbours[:1]
    return sum( (n1, n2) == (0, 1) for n1, n2 in zip(neighbours, neighbours[1:]) )

# 黒を1、白を0とする画像を、2値画像に戻すメソッドです
def inv_black_one(inv_bool_image):
    bool_image = ~inv_bool_image.astype(bool)
    return bool_image.astype(int) * 255


def display_result_image(cap, color_image, skeleton):
    colorimg = color_image.copy()

    # カラー画像に細線化を合成
    colorimg = colorimg // 2 + 127
    colorimg[skeleton == 255] = 0

    cv2.imshow(cap + '_skeleton', skeleton)
    cv2.imshow(cap + '_color image', colorimg)
    cv2.waitKey(0)

def FLD(image):
    # Create default Fast Line Detector class
    fld = cv2.ximgproc.createFastLineDetector()
    # Get line vectors from the image
    lines = fld.detect(image)
    # Draw lines on the image
    line_on_image = fld.drawSegments(image, lines)
    # Plot
    plt.imshow(line_on_image, interpolation='nearest', aspect='auto')
    plt.show()
    return line_on_image

def canny(image_name,min,max):
    img = cv2.imread(image_name, cv2.IMREAD_GRAYSCALE)
    #img=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(img,min,max)
    return edges




def disp(img1,img2):
    plt.subplot(121),plt.imshow(img1,cmap = 'gray')
    plt.title('Original Image'), plt.xticks([]), plt.yticks([])
    plt.subplot(122),plt.imshow(img2,cmap = 'gray')
    plt.title('Edge Image'), plt.xticks([]), plt.yticks([])

    plt.show()

#img=cv2.imread('debug.png')
#gray=cv2.cvtColor(img,cv2.COLOR_RGB2GRAY)
#edges = cv2.Canny(gray,0,100)
##edges=canny('debug2.png',0,100)


#lines = cv2.HoughLines(edges,1,np.pi/180,10)
#OutputController().msgPrint("line are",len(lines))
#counter=0

#for counter in range(20):
#    rho,theta=lines[0][0]
#    a = np.cos(theta)
#    b = np.sin(theta)
#    x0 = a*rho
#    y0 = b*rho
#    x1 = int(x0 + 1000*(-b))
#    y1 = int(y0 + 1000*(a))
#    x2 = int(x0 - 1000*(-b))
#    y2 = int(y0 - 1000*(a))
#    cv2.line(img,(x1,y1),(x2,y2),(0,255,0),2)
#    OutputController().msgPrint(rho,theta)
#    lines=lines[np.where((lines[:,0,0]>rho*1.2)|(lines[:,0,0]<rho*0.8)|(lines[:,0,1]>theta*1.2)|(lines[:,0,1]<theta*0.8))[0]]
#    lines=lines[np.where((lines[:,0,0]<rho*1.2)|(lines[:,0,0]>rho*0.8)|(lines[:,0,1]>theta*1.2)|(lines[:,0,1]<theta*0.8))[0]]#rhoは負数を取りうる
#    OutputController().msgPrint(len(lines))
    

##disp(img,edges)

#img=cv2.imread('debug3.png')
#gray=cv2.cvtColor(img,cv2.COLOR_RGB2GRAY)
#edges = cv2.Canny(gray,0,100)
##edges=canny('debug2.png',0,100)


#canny('debug.png',0,100)
#canny('debug2.png',0,100)


# 画像を読み込みます
#image = cv2.imread('debug.png')
# 入力画像の取得
colorimg = cv2.imread('debug.png', cv2.IMREAD_COLOR)

# グレースケール変換
gray = cv2.cvtColor(colorimg, cv2.COLOR_BGR2GRAY)
#gray = cv2.GaussianBlur(gray,(5,5), 3)
image = cv2.Canny(gray,0,100)
#_, image = cv2.threshold(gray, 30, 255, cv2.THRESH_BINARY)

# 二値画像反転
image = cv2.bitwise_not(image)
#disp(gray,image)

ske = skeletonize(~(image != 0))
ske_gray = (ske * 255).astype(np.uint8)
ske_rgb = cv2.cvtColor(ske_gray, cv2.COLOR_GRAY2RGB)
#display_result_image('GUOHALL', colorimg, ske_rgb)


edges = cv2.Canny(ske_rgb,0,100)
lines = cv2.HoughLines(edges,1,np.pi/180,10)
OutputController().msgPrint("line are",len(lines))
counter=0
draw_lines=np.zeros(colorimg.shape)

for counter in range(10):
    if(len(lines)==0):
        break;
    rho,theta=lines[0][0]
    a = np.cos(theta)
    b = np.sin(theta)
    x0 = a*rho
    y0 = b*rho
    x1 = int(x0 + 1000*(-b))
    y1 = int(y0 + 1000*(a))
    x2 = int(x0 - 1000*(-b))
    y2 = int(y0 - 1000*(a))
    cv2.line(draw_lines,(x1,y1),(x2,y2),(0,255,0),2)
    OutputController().msgPrint(rho,theta)
    lines=lines[np.where((lines[:,0,0]<rho-30)|(lines[:,0,0]>rho+30)|(lines[:,0,1]<theta-3.14/8)|(lines[:,0,1]>theta+3.14/8))[0]]
    #lines=lines[np.where((lines[:,0,0]<rho*1.2)|(lines[:,0,0]>rho*0.8)|(lines[:,0,1]>theta*1.2)|(lines[:,0,1]<theta*0.8))[0]]#rhoは負数を取りうる
    OutputController().msgPrint(len(lines))


kernel = np.ones((5,5),np.uint8)
dilation = cv2.dilate(draw_lines,kernel,iterations = 1)
#disp(draw_lines,dilation)

disp(colorimg,cv2.addWeighted(colorimg,0.5,dilation,0.5,0,dtype=cv2.CV_8UC3))
