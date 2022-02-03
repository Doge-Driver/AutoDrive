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

class freetype_FreeType2(Algorithm):
    # no doc
    def getTextSize(self, text, fontHeight, thickness): # real signature unknown; restored from __doc__
        """
        getTextSize(text, fontHeight, thickness) -> retval, baseLine
        .   @brief Calculates the width and height of a text string.
        .   
        .   The function getTextSize calculates and returns the approximate size of a box that contains the specified text.
        .   That is, the following code renders some text, the tight box surrounding it, and the baseline: :
        .   @code
        .       String text = "Funny text inside the box";
        .       int fontHeight = 60;
        .       int thickness = -1;
        .       int linestyle = 8;
        .   
        .       Mat img(600, 800, CV_8UC3, Scalar::all(0));
        .   
        .       int baseline=0;
        .   
        .       cv::Ptr<cv::freetype::FreeType2> ft2;
        .       ft2 = cv::freetype::createFreeType2();
        .       ft2->loadFontData( "./mplus-1p-regular.ttf", 0 );
        .   
        .       Size textSize = ft2->getTextSize(text,
        .                                        fontHeight,
        .                                        thickness,
        .                                        &baseline);
        .   
        .       if(thickness > 0){
        .           baseline += thickness;
        .       }
        .   
        .       // center the text
        .       Point textOrg((img.cols - textSize.width) / 2,
        .                     (img.rows + textSize.height) / 2);
        .   
        .       // draw the box
        .       rectangle(img, textOrg + Point(0, baseline),
        .                 textOrg + Point(textSize.width, -textSize.height),
        .                 Scalar(0,255,0),1,8);
        .   
        .       // ... and the baseline first
        .       line(img, textOrg + Point(0, thickness),
        .            textOrg + Point(textSize.width, thickness),
        .            Scalar(0, 0, 255),1,8);
        .   
        .       // then put the text itself
        .       ft2->putText(img, text, textOrg, fontHeight,
        .                    Scalar::all(255), thickness, linestyle, true );
        .   @endcode
        .   
        .   @param text Input text string.
        .   @param fontHeight Drawing font size by pixel unit.
        .   @param thickness Thickness of lines used to render the text. See putText for details.
        .   @param[out] baseLine y-coordinate of the baseline relative to the bottom-most text
        .   point.
        .   @return The size of a box that contains the specified text.
        .   
        .   @see cv::putText
        """
        pass

    def loadFontData(self, fontFileName, id): # real signature unknown; restored from __doc__
        """
        loadFontData(fontFileName, id) -> None
        .   @brief Load font data.
        .   
        .   The function loadFontData loads font data.
        .   
        .   @param fontFileName FontFile Name
        .   @param id face_index to select a font faces in a single file.
        """
        pass

    def putText(self, img, text, org, fontHeight, color, thickness, line_type, bottomLeftOrigin): # real signature unknown; restored from __doc__
        """
        putText(img, text, org, fontHeight, color, thickness, line_type, bottomLeftOrigin) -> img
        .   @brief Draws a text string.
        .   
        .   The function putText renders the specified text string in the image. Symbols that cannot be rendered using the specified font are replaced by "Tofu" or non-drawn.
        .   
        .   @param img Image.
        .   @param text Text string to be drawn.
        .   @param org Bottom-left/Top-left corner of the text string in the image.
        .   @param fontHeight Drawing font size by pixel unit.
        .   @param color Text color.
        .   @param thickness Thickness of the lines used to draw a text when negative, the glyph is filled. Otherwise, the glyph is drawn with this thickness.
        .   @param line_type Line type. See the line for details.
        .   @param bottomLeftOrigin When true, the image data origin is at the bottom-left corner. Otherwise, it is at the top-left corner.
        """
        pass

    def setSplitNumber(self, num): # real signature unknown; restored from __doc__
        """
        setSplitNumber(num) -> None
        .   @brief Set Split Number from Bezier-curve to line
        .   
        .   The function setSplitNumber set the number of split points from bezier-curve to line.
        .   If you want to draw large glyph, large is better.
        .   If you want to draw small glyph, small is better.
        .   
        .   @param num number of split points from bezier-curve to line
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


