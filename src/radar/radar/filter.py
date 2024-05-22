import cv2
import numpy as np

MAX_SPOKE_LENGTH = 256

def generate_map(r, k, p, K, area_threshold, gamma):
    """
    Args:
        r (_type_): raw radar data
        k (_type_): spline curve degree
        p (_type_): knot spacing
        K (_type_): coordinate transformation matrix
        area_threshold (_type_): minimum polygon area threshold
        gamma (_type_): angular resolution to discretize the polar coordinate 
    """
    I = generate_radar_image(r, K)
    D = detect_contour(filter_image(I,binary_threshold=128))
    P = extract_coastline(D, area_threshold=10, angular_resolution=None, K=None)
    
    return I, D, P


def generate_radar_image(spokes, K):
    """convert 2d polar image of radar data to 2d cartesian imagecv2.im

    Args:
        spokes (_type_): 2D array of dimension (num_spokes, MAX_SPOKE_LENGTH)
                                with each row representing a single radar spoke

    Returns:
        radar_image (_type_): grayscale image of radar scan in cartesian coordinates
    """
    radar_image = cv2.warpPolar(
        src=spokes, 
        dsize=(2*MAX_SPOKE_LENGTH, 2*MAX_SPOKE_LENGTH), 
        center=(MAX_SPOKE_LENGTH, MAX_SPOKE_LENGTH), 
        maxRadius=MAX_SPOKE_LENGTH, flags=cv2.WARP_INVERSE_MAP)
    radar_image = cv2.rotate(radar_image, cv2.ROTATE_90_CLOCKWISE)
    
    # get_logger().info(f'{radar_image=} {radar_image.shape}')
    # cv2.imshow('raw image', radar_image); cv2.waitKey(0)
    cv2.imwrite('test/radar_image.jpg', radar_image)
    
    return radar_image


def filter_image(radar_image, binary_threshold):
    """apply morphological and bilateral filters for denoising
    and convert grayscale radar_image to binary image according to 
    predetermined threshold intensity value
    
    Args:
        radar_image (_type_): grayscale image of radar scan in cartesian coordinates

    Returns:
        _type_: _description_
    """
    
    # [TODO]: apply morphological and bilateral filters
    erosion = cv2.erode(radar_image, np.ones((3, 3), np.uint8), iterations=1)
    # cv2.imshow('erosion', erosion); cv2.waitKey(0)
    cv2.imwrite('test/erosion.jpg', erosion)
    dilation = cv2.dilate(erosion, np.ones((3, 3), np.uint8), iterations=1)
    # cv2.imshow('dilation', dilation); cv2.waitKey(0)
    cv2.imwrite('test/dilation.jpg', dilation)
    
    bilateral = cv2.bilateralFilter(dilation, 9, 100, 100)
    # cv2.imshow('bilateral', bilateral); cv2.waitKey(0)
    cv2.imwrite('test/bilateral.jpg', bilateral)
    
    # [TODO]: adjust threshold intensity value. range:[0,255]
    _, binary_image = cv2.threshold(bilateral, binary_threshold, 255, cv2.THRESH_BINARY)
    # cv2.imshow('threshold', binary_image); cv2.waitKey(0)
    cv2.imwrite('test/binary_image.jpg', binary_image)
    
    return binary_image


def detect_contour(binary_image):
    """extract the contours from the binary image using polygon extraction

    Args:
        binary_image (_type_): _description_

    Returns:
        _type_: _description_
    """
    # [TODO]: extract the contours from the binary image using polygon extraction
    
    contours, hierarchy = cv2.findContours(binary_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    return contours


def extract_coastline(contours, area_threshold, angular_resolution=None, K=None):
    landmasses = [cnt for cnt in contours if cv2.contourArea(cnt) > area_threshold]
    
    contour = np.zeros((2*MAX_SPOKE_LENGTH, 2*MAX_SPOKE_LENGTH, 3), dtype=np.uint8)
    cv2.drawContours(contour, landmasses, -1, (255,255,255), -1)
    # cv2.imshow('contour', contour); cv2.waitKey(0)
    cv2.imwrite('test/contour.jpg', contour)
    
    height, width = contour.shape[:2]
    polar = cv2.warpPolar(src=contour, dsize=(MAX_SPOKE_LENGTH, 0), center=(width/2, height/2), maxRadius=width/2, flags=cv2.WARP_POLAR_LINEAR)
    polar = cv2.cvtColor(polar, cv2.COLOR_BGR2GRAY)
    # cv2.imshow('polar', polar); cv2.waitKey(0)
    
    for spoke in polar:
        spoke[np.argmax(spoke > 0)+1:] = 0
    
    # cv2.imshow('coastline polar', polar); cv2.waitKey(0)
    
    coastline = cv2.warpPolar(
        src=polar, 
        dsize=(2*MAX_SPOKE_LENGTH, 2*MAX_SPOKE_LENGTH), 
        center=(MAX_SPOKE_LENGTH, MAX_SPOKE_LENGTH), 
        maxRadius=MAX_SPOKE_LENGTH, flags=cv2.WARP_INVERSE_MAP)
    
    # cv2.imshow('coastline', coastline); cv2.waitKey(0)
    cv2.imwrite('test/coastline.jpg', coastline)
    
    return coastline
