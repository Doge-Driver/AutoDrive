# encoding: utf-8
# module cv2
# from /usr/lib/python3/dist-packages/cv2.cpython-38-x86_64-linux-gnu.so
# by generator 1.147
""" Python wrapper for OpenCV. """

# imports
import cv2 as  # <module 'cv2'>
import cv2.Error as Error # <module 'cv2.Error'>
import cv2.aruco as aruco # <module 'cv2.aruco'>
import cv2.bgsegm as bgsegm # <module 'cv2.bgsegm'>
import cv2.bioinspired as bioinspired # <module 'cv2.bioinspired'>
import cv2.cuda as cuda # <module 'cv2.cuda'>
import cv2.datasets as datasets # <module 'cv2.datasets'>
import cv2.detail as detail # <module 'cv2.detail'>
import cv2.dnn as dnn # <module 'cv2.dnn'>
import cv2.dynafu as dynafu # <module 'cv2.dynafu'>
import cv2.face as face # <module 'cv2.face'>
import cv2.fisheye as fisheye # <module 'cv2.fisheye'>
import cv2.flann as flann # <module 'cv2.flann'>
import cv2.freetype as freetype # <module 'cv2.freetype'>
import cv2.ft as ft # <module 'cv2.ft'>
import cv2.hdf as hdf # <module 'cv2.hdf'>
import cv2.hfs as hfs # <module 'cv2.hfs'>
import cv2.img_hash as img_hash # <module 'cv2.img_hash'>
import cv2.ipp as ipp # <module 'cv2.ipp'>
import cv2.kinfu as kinfu # <module 'cv2.kinfu'>
import cv2.line_descriptor as line_descriptor # <module 'cv2.line_descriptor'>
import cv2.linemod as linemod # <module 'cv2.linemod'>
import cv2.ml as ml # <module 'cv2.ml'>
import cv2.motempl as motempl # <module 'cv2.motempl'>
import cv2.multicalib as multicalib # <module 'cv2.multicalib'>
import cv2.ocl as ocl # <module 'cv2.ocl'>
import cv2.ogl as ogl # <module 'cv2.ogl'>
import cv2.omnidir as omnidir # <module 'cv2.omnidir'>
import cv2.optflow as optflow # <module 'cv2.optflow'>
import cv2.plot as plot # <module 'cv2.plot'>
import cv2.ppf_match_3d as ppf_match_3d # <module 'cv2.ppf_match_3d'>
import cv2.quality as quality # <module 'cv2.quality'>
import cv2.reg as reg # <module 'cv2.reg'>
import cv2.rgbd as rgbd # <module 'cv2.rgbd'>
import cv2.saliency as saliency # <module 'cv2.saliency'>
import cv2.samples as samples # <module 'cv2.samples'>
import cv2.structured_light as structured_light # <module 'cv2.structured_light'>
import cv2.text as text # <module 'cv2.text'>
import cv2.utils as utils # <module 'cv2.utils'>
import cv2.videoio_registry as videoio_registry # <module 'cv2.videoio_registry'>
import cv2.videostab as videostab # <module 'cv2.videostab'>
import cv2.viz as viz # <module 'cv2.viz'>
import cv2.ximgproc as ximgproc # <module 'cv2.ximgproc'>
import cv2.xphoto as xphoto # <module 'cv2.xphoto'>

from .Algorithm import Algorithm

class rgbd_RgbdPlane(Algorithm):
    # no doc
    def apply(self, points3d, normals, mask=None, plane_coefficients=None): # real signature unknown; restored from __doc__
        """
        apply(points3d, normals[, mask[, plane_coefficients]]) -> mask, plane_coefficients
        .   Find The planes in a depth image
        .        * @param points3d the 3d points organized like the depth image: rows x cols with 3 channels
        .        * @param normals the normals for every point in the depth image
        .        * @param mask An image where each pixel is labeled with the plane it belongs to
        .        *        and 255 if it does not belong to any plane
        .        * @param plane_coefficients the coefficients of the corresponding planes (a,b,c,d) such that ax+by+cz+d=0, norm(a,b,c)=1
        .        *        and c < 0 (so that the normal points towards the camera)
        
        
        
        apply(points3d[, mask[, plane_coefficients]]) -> mask, plane_coefficients
        .   Find The planes in a depth image but without doing a normal check, which is faster but less accurate
        .        * @param points3d the 3d points organized like the depth image: rows x cols with 3 channels
        .        * @param mask An image where each pixel is labeled with the plane it belongs to
        .        *        and 255 if it does not belong to any plane
        .        * @param plane_coefficients the coefficients of the corresponding planes (a,b,c,d) such that ax+by+cz+d=0
        """
        pass

    def create(self, method, block_size, min_size, threshold, sensor_error_a=None, sensor_error_b=None, sensor_error_c=None): # real signature unknown; restored from __doc__
        """
        create(method, block_size, min_size, threshold[, sensor_error_a[, sensor_error_b[, sensor_error_c]]]) -> retval
        .   Constructor
        .        * @param block_size The size of the blocks to look at for a stable MSE
        .        * @param min_size The minimum size of a cluster to be considered a plane
        .        * @param threshold The maximum distance of a point from a plane to belong to it (in meters)
        .        * @param sensor_error_a coefficient of the sensor error. 0 by default, 0.0075 for a Kinect
        .        * @param sensor_error_b coefficient of the sensor error. 0 by default
        .        * @param sensor_error_c coefficient of the sensor error. 0 by default
        .        * @param method The method to use to compute the planes.
        """
        pass

    def getBlockSize(self): # real signature unknown; restored from __doc__
        """
        getBlockSize() -> retval
        .
        """
        pass

    def getMethod(self): # real signature unknown; restored from __doc__
        """
        getMethod() -> retval
        .
        """
        pass

    def getMinSize(self): # real signature unknown; restored from __doc__
        """
        getMinSize() -> retval
        .
        """
        pass

    def getSensorErrorA(self): # real signature unknown; restored from __doc__
        """
        getSensorErrorA() -> retval
        .
        """
        pass

    def getSensorErrorB(self): # real signature unknown; restored from __doc__
        """
        getSensorErrorB() -> retval
        .
        """
        pass

    def getSensorErrorC(self): # real signature unknown; restored from __doc__
        """
        getSensorErrorC() -> retval
        .
        """
        pass

    def getThreshold(self): # real signature unknown; restored from __doc__
        """
        getThreshold() -> retval
        .
        """
        pass

    def setBlockSize(self, val): # real signature unknown; restored from __doc__
        """
        setBlockSize(val) -> None
        .
        """
        pass

    def setMethod(self, val): # real signature unknown; restored from __doc__
        """
        setMethod(val) -> None
        .
        """
        pass

    def setMinSize(self, val): # real signature unknown; restored from __doc__
        """
        setMinSize(val) -> None
        .
        """
        pass

    def setSensorErrorA(self, val): # real signature unknown; restored from __doc__
        """
        setSensorErrorA(val) -> None
        .
        """
        pass

    def setSensorErrorB(self, val): # real signature unknown; restored from __doc__
        """
        setSensorErrorB(val) -> None
        .
        """
        pass

    def setSensorErrorC(self, val): # real signature unknown; restored from __doc__
        """
        setSensorErrorC(val) -> None
        .
        """
        pass

    def setThreshold(self, val): # real signature unknown; restored from __doc__
        """
        setThreshold(val) -> None
        .
        """
        pass

    def __init__(self, *args, **kwargs): # real signature unknown
        pass

    @staticmethod # known case of __new__
    def __new__(*args, **kwargs): # real signature unknown
        """ Create and return a new object.  See help(type) for accurate signature. """
        pass

    def __repr__(self, *args, **kwargs): # real signature unknown
        """ Return repr(self). """
        pass


