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

from .ximgproc_SparseMatchInterpolator import ximgproc_SparseMatchInterpolator

class ximgproc_RICInterpolator(ximgproc_SparseMatchInterpolator):
    # no doc
    def getAlpha(self): # real signature unknown; restored from __doc__
        """
        getAlpha() -> retval
        .   @copybrief setAlpha
        .        *  @see setAlpha
        """
        pass

    def getFGSLambda(self): # real signature unknown; restored from __doc__
        """
        getFGSLambda() -> retval
        .   @copybrief setFGSLambda
        .        *  @see setFGSLambda
        """
        pass

    def getFGSSigma(self): # real signature unknown; restored from __doc__
        """
        getFGSSigma() -> retval
        .   @copybrief setFGSSigma
        .        *  @see setFGSSigma
        """
        pass

    def getK(self): # real signature unknown; restored from __doc__
        """
        getK() -> retval
        .   @copybrief setK
        .        *  @see setK
        """
        pass

    def getMaxFlow(self): # real signature unknown; restored from __doc__
        """
        getMaxFlow() -> retval
        .   @copybrief setMaxFlow
        .        *  @see setMaxFlow
        """
        pass

    def getModelIter(self): # real signature unknown; restored from __doc__
        """
        getModelIter() -> retval
        .   @copybrief setModelIter
        .        *  @see setModelIter
        """
        pass

    def getRefineModels(self): # real signature unknown; restored from __doc__
        """
        getRefineModels() -> retval
        .   @copybrief setRefineModels
        .        *  @see setRefineModels
        """
        pass

    def getSuperpixelMode(self): # real signature unknown; restored from __doc__
        """
        getSuperpixelMode() -> retval
        .   @copybrief setSuperpixelMode
        .        *  @see setSuperpixelMode
        """
        pass

    def getSuperpixelNNCnt(self): # real signature unknown; restored from __doc__
        """
        getSuperpixelNNCnt() -> retval
        .   @copybrief setSuperpixelNNCnt
        .        *  @see setSuperpixelNNCnt
        """
        pass

    def getSuperpixelRuler(self): # real signature unknown; restored from __doc__
        """
        getSuperpixelRuler() -> retval
        .   @copybrief setSuperpixelRuler
        .        *  @see setSuperpixelRuler
        """
        pass

    def getSuperpixelSize(self): # real signature unknown; restored from __doc__
        """
        getSuperpixelSize() -> retval
        .   @copybrief setSuperpixelSize
        .        *  @see setSuperpixelSize
        """
        pass

    def getUseGlobalSmootherFilter(self): # real signature unknown; restored from __doc__
        """
        getUseGlobalSmootherFilter() -> retval
        .   @copybrief setUseGlobalSmootherFilter
        .        *  @see setUseGlobalSmootherFilter
        """
        pass

    def getUseVariationalRefinement(self): # real signature unknown; restored from __doc__
        """
        getUseVariationalRefinement() -> retval
        .   @copybrief setUseVariationalRefinement
        .        *  @see setUseVariationalRefinement
        """
        pass

    def setAlpha(self, alpha=None): # real signature unknown; restored from __doc__
        """
        setAlpha([, alpha]) -> None
        .   @brief Alpha is a parameter defining a global weight for transforming geodesic distance into weight.
        """
        pass

    def setCostMap(self, costMap): # real signature unknown; restored from __doc__
        """
        setCostMap(costMap) -> None
        .   @brief Interface to provide a more elaborated cost map, i.e. edge map, for the edge-aware term.
        .        *  This implementation is based on a rather simple gradient-based edge map estimation.
        .        *  To used more complex edge map estimator (e.g. StructuredEdgeDetection that has been
        .        *  used in the original publication) that may lead to improved accuracies, the internal
        .        *  edge map estimation can be bypassed here.
        .        *  @param costMap a type CV_32FC1 Mat is required.
        .        *  @see cv::ximgproc::createSuperpixelSLIC
        """
        pass

    def setFGSLambda(self, lambda=None): # real signature unknown; restored from __doc__
        """
        setFGSLambda([, lambda]) -> None
        .   @brief Sets the respective fastGlobalSmootherFilter() parameter.
        """
        pass

    def setFGSSigma(self, sigma=None): # real signature unknown; restored from __doc__
        """
        setFGSSigma([, sigma]) -> None
        .   @brief Sets the respective fastGlobalSmootherFilter() parameter.
        """
        pass

    def setK(self, k=None): # real signature unknown; restored from __doc__
        """
        setK([, k]) -> None
        .   @brief K is a number of nearest-neighbor matches considered, when fitting a locally affine
        .        *model for a superpixel segment. However, lower values would make the interpolation
        .        *noticeably faster. The original implementation of @cite Hu2017 uses 32.
        """
        pass

    def setMaxFlow(self, maxFlow=None): # real signature unknown; restored from __doc__
        """
        setMaxFlow([, maxFlow]) -> None
        .   @brief MaxFlow is a threshold to validate the predictions using a certain piece-wise affine model.
        .        * If the prediction exceeds the treshold the translational model will be applied instead.
        """
        pass

    def setModelIter(self, modelIter=None): # real signature unknown; restored from __doc__
        """
        setModelIter([, modelIter]) -> None
        .   @brief Parameter defining the number of iterations for piece-wise affine model estimation.
        """
        pass

    def setRefineModels(self, refineModles=None): # real signature unknown; restored from __doc__
        """
        setRefineModels([, refineModles]) -> None
        .   @brief Parameter to choose wether additional refinement of the piece-wise affine models is employed.
        """
        pass

    def setSuperpixelMode(self, mode=None): # real signature unknown; restored from __doc__
        """
        setSuperpixelMode([, mode]) -> None
        .   @brief Parameter to choose superpixel algorithm variant to use:
        .        * - cv::ximgproc::SLICType SLIC segments image using a desired region_size (value: 100)
        .        * - cv::ximgproc::SLICType SLICO will optimize using adaptive compactness factor (value: 101)
        .        * - cv::ximgproc::SLICType MSLIC will optimize using manifold methods resulting in more content-sensitive superpixels (value: 102).
        .        *  @see cv::ximgproc::createSuperpixelSLIC
        """
        pass

    def setSuperpixelNNCnt(self, spNN=None): # real signature unknown; restored from __doc__
        """
        setSuperpixelNNCnt([, spNN]) -> None
        .   @brief Parameter defines the number of nearest-neighbor matches for each superpixel considered, when fitting a locally affine
        .        *model.
        """
        pass

    def setSuperpixelRuler(self, ruler=None): # real signature unknown; restored from __doc__
        """
        setSuperpixelRuler([, ruler]) -> None
        .   @brief Parameter to tune enforcement of superpixel smoothness factor used for oversegmentation.
        .        *  @see cv::ximgproc::createSuperpixelSLIC
        """
        pass

    def setSuperpixelSize(self, spSize=None): # real signature unknown; restored from __doc__
        """
        setSuperpixelSize([, spSize]) -> None
        .   @brief Get the internal cost, i.e. edge map, used for estimating the edge-aware term.
        .        *  @see setCostMap
        """
        pass

    def setUseGlobalSmootherFilter(self, use_FGS=None): # real signature unknown; restored from __doc__
        """
        setUseGlobalSmootherFilter([, use_FGS]) -> None
        .   @brief Sets whether the fastGlobalSmootherFilter() post-processing is employed.
        """
        pass

    def setUseVariationalRefinement(self, use_variational_refinement=None): # real signature unknown; restored from __doc__
        """
        setUseVariationalRefinement([, use_variational_refinement]) -> None
        .   @brief Parameter to choose wether the VariationalRefinement post-processing  is employed.
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


