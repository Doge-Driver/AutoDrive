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

from .object import object

class hdf_HDF5(object):
    # no doc
    def atdelete(self, atlabel): # real signature unknown; restored from __doc__
        """
        atdelete(atlabel) -> None
        .   * Delete an attribute from the root group.
        .        *
        .        * @param atlabel the attribute to be deleted.
        .        *
        .        * @note CV_Error() is called if the given attribute does not exist. Use atexists()
        .        * to check whether it exists or not beforehand.
        .        *
        .        * @sa atexists, atwrite, atread
        """
        pass

    def atexists(self, atlabel): # real signature unknown; restored from __doc__
        """
        atexists(atlabel) -> retval
        .   * Check whether a given attribute exits or not in the root group.
        .        *
        .        * @param atlabel the attribute name to be checked.
        .        * @return true if the attribute exists, false otherwise.
        .        *
        .        * @sa atdelete, atwrite, atread
        """
        pass

    def atread(self, value, atlabel): # real signature unknown; restored from __doc__
        """
        atread(value, atlabel) -> None
        .   * Read an attribute from the root group.
        .        *
        .        * @param value address where the attribute is read into
        .        * @param atlabel attribute name
        .        *
        .        * The following example demonstrates how to read an attribute of type cv::String:
        .        *
        .        *  @snippet samples/read_write_attributes.cpp snippets_read_str
        .        *
        .        * @note The attribute MUST exist, otherwise CV_Error() is called. Use atexists()
        .        * to check if it exists beforehand.
        .        *
        .        * @sa atexists, atdelete, atwrite
        
        
        
        atread(atlabel[, value]) -> value
        .   * Read an attribute from the root group.
        .        *
        .        * @param value attribute value. Currently, only n-d continuous multi-channel arrays are supported.
        .        * @param atlabel attribute name.
        .        *
        .        * @note The attribute MUST exist, otherwise CV_Error() is called. Use atexists()
        .        * to check if it exists beforehand.
        .        *
        .        * @sa atexists, atdelete, atwrite
        """
        pass

    def atwrite(self, value, atlabel): # real signature unknown; restored from __doc__
        """
        atwrite(value, atlabel) -> None
        .   * Write an attribute inside the root group.
        .        *
        .        * @param value attribute value.
        .        * @param atlabel attribute name.
        .        *
        .        * The following example demonstrates how to write an attribute of type cv::String:
        .        *
        .        *  @snippet samples/read_write_attributes.cpp snippets_write_str
        .        *
        .        * @note CV_Error() is called if the given attribute already exists. Use atexists()
        .        * to check whether it exists or not beforehand. And use atdelete() to delete
        .        * it if it already exists.
        .        *
        .        * @sa atexists, atdelete, atread
        """
        pass

    def close(self): # real signature unknown; restored from __doc__
        """
        close() -> None
        .   @brief Close and release hdf5 object.
        """
        pass

    def dscreate(self, rows, cols, type, dslabel): # real signature unknown; restored from __doc__
        """
        dscreate(rows, cols, type, dslabel) -> None
        .   @overload
        
        
        
        dscreate(rows, cols, type, dslabel, compresslevel) -> None
        .   @overload
        
        
        
        dscreate(rows, cols, type, dslabel, compresslevel, dims_chunks) -> None
        .   @overload
        
        
        
        dscreate(n_dims, sizes, type, dslabel) -> None
        .   
        
        
        
        dscreate(n_dims, sizes, type, dslabel, compresslevel) -> None
        .   
        
        
        
        dscreate(sizes, type, dslabel[, compresslevel[, dims_chunks]]) -> None
        .   
        
        
        
        dscreate(n_dims, sizes, type, dslabel, compresslevel, dims_chunks) -> None
        .   @brief Create and allocate storage for n-dimensional dataset, single or multichannel type.
        .       @param n_dims declare number of dimensions
        .       @param sizes array containing sizes for each dimensions
        .       @param type type to be used, e.g., CV_8UC3, CV_32FC1, etc.
        .       @param dslabel specify the hdf5 dataset label. Existing dataset label will cause an error.
        .       @param compresslevel specify the compression level 0-9 to be used, H5_NONE is the default value and means no compression.
        .                            The value 0 also means no compression.
        .                            A value 9 indicating the best compression ration. Note
        .                            that a higher compression level indicates a higher computational cost. It relies
        .                            on GNU gzip for compression.
        .       @param dims_chunks each array member specifies chunking sizes to be used for block I/O,
        .              by default NULL means none at all.
        .       @note If the dataset already exists, an exception will be thrown. Existence of the dataset can be checked
        .       using hlexists().
        .   
        .       - See example below that creates a 6 dimensional storage space:
        .       @code{.cpp}
        .         // open / autocreate hdf5 file
        .         cv::Ptr<cv::hdf::HDF5> h5io = cv::hdf::open( "mytest.h5" );
        .         // create space for 6 dimensional CV_64FC2 matrix
        .         if ( ! h5io->hlexists( "nddata" ) )
        .           int n_dims = 5;
        .           int dsdims[n_dims] = { 100, 100, 20, 10, 5, 5 };
        .           h5io->dscreate( n_dims, sizes, CV_64FC2, "nddata" );
        .         else
        .           printf("DS already created, skipping\n" );
        .         // release
        .         h5io->close();
        .       @endcode
        .   
        .       @note Activating compression requires internal chunking. Chunking can significantly improve access
        .       speed both at read and write time, especially for windowed access logic that shifts offset inside dataset.
        .       If no custom chunking is specified, the default one will be invoked by the size of **whole** dataset
        .       as single big chunk of data.
        .   
        .       - See example of level 0 compression (shallow) using chunking against the first
        .       dimension, thus storage will consists of 100 chunks of data:
        .       @code{.cpp}
        .         // open / autocreate hdf5 file
        .         cv::Ptr<cv::hdf::HDF5> h5io = cv::hdf::open( "mytest.h5" );
        .         // create space for 6 dimensional CV_64FC2 matrix
        .         if ( ! h5io->hlexists( "nddata" ) )
        .           int n_dims = 5;
        .           int dsdims[n_dims] = { 100, 100, 20, 10, 5, 5 };
        .           int chunks[n_dims] = {   1, 100, 20, 10, 5, 5 };
        .           h5io->dscreate( n_dims, dsdims, CV_64FC2, "nddata", 0, chunks );
        .         else
        .           printf("DS already created, skipping\n" );
        .         // release
        .         h5io->close();
        .       @endcode
        .   
        .       @note A value of H5_UNLIMITED inside the **sizes** array means **unlimited** data on that dimension, thus it is
        .       possible to expand anytime such dataset on those unlimited directions. Presence of H5_UNLIMITED on any dimension
        .       **requires** to define custom chunking. No default chunking will be defined in unlimited scenario since the default size
        .       on that dimension will be zero, and will grow once dataset is written. Writing into dataset that has H5_UNLIMITED on
        .       some of its dimension requires dsinsert() instead of dswrite() that allows growth on unlimited dimension instead of
        .       dswrite() that allows to write only in predefined data space.
        .   
        .       - Example below shows a 3 dimensional dataset using no compression with all unlimited sizes and one unit chunking:
        .       @code{.cpp}
        .         // open / autocreate hdf5 file
        .         cv::Ptr<cv::hdf::HDF5> h5io = cv::hdf::open( "mytest.h5" );
        .         int n_dims = 3;
        .         int chunks[n_dims] = { 1, 1, 1 };
        .         int dsdims[n_dims] = { cv::hdf::HDF5::H5_UNLIMITED, cv::hdf::HDF5::H5_UNLIMITED, cv::hdf::HDF5::H5_UNLIMITED };
        .         h5io->dscreate( n_dims, dsdims, CV_64FC2, "nddata", cv::hdf::HDF5::H5_NONE, chunks );
        .         // release
        .         h5io->close();
        .       @endcode
        """
        pass

    def dsgetsize(self, dslabel, dims_flag=None): # real signature unknown; restored from __doc__
        """
        dsgetsize(dslabel[, dims_flag]) -> retval
        .   @brief Fetch dataset sizes
        .       @param dslabel specify the hdf5 dataset label to be measured.
        .       @param dims_flag will fetch dataset dimensions on H5_GETDIMS, dataset maximum dimensions on H5_GETMAXDIMS,
        .                        and chunk sizes on H5_GETCHUNKDIMS.
        .   
        .       Returns vector object containing sizes of dataset on each dimensions.
        .   
        .       @note Resulting vector size will match the amount of dataset dimensions. By default H5_GETDIMS will return
        .       actual dataset dimensions. Using H5_GETMAXDIM flag will get maximum allowed dimension which normally match
        .       actual dataset dimension but can hold H5_UNLIMITED value if dataset was prepared in **unlimited** mode on
        .       some of its dimension. It can be useful to check existing dataset dimensions before overwrite it as whole or subset.
        .       Trying to write with oversized source data into dataset target will thrown exception. The H5_GETCHUNKDIMS will
        .       return the dimension of chunk if dataset was created with chunking options otherwise returned vector size
        .       will be zero.
        """
        pass

    def dsgettype(self, dslabel): # real signature unknown; restored from __doc__
        """
        dsgettype(dslabel) -> retval
        .   @brief Fetch dataset type
        .       @param dslabel specify the hdf5 dataset label to be checked.
        .   
        .       Returns the stored matrix type. This is an identifier compatible with the CvMat type system,
        .       like e.g. CV_16SC5 (16-bit signed 5-channel array), and so on.
        .   
        .       @note Result can be parsed with CV_MAT_CN() to obtain amount of channels and CV_MAT_DEPTH() to obtain native cvdata type.
        .       It is thread safe.
        """
        pass

    def dsinsert(self, Array, dslabel): # real signature unknown; restored from __doc__
        """
        dsinsert(Array, dslabel) -> None
        .   
        
        
        
        dsinsert(Array, dslabel, dims_offset) -> None
        .   
        
        
        
        dsinsert(Array, dslabel, dims_offset[, dims_counts]) -> None
        .   
        
        
        
        dsinsert(Array, dslabel, dims_offset, dims_counts) -> None
        .   @brief Insert or overwrite a Mat object into specified dataset and auto expand dataset size if **unlimited** property allows.
        .       @param Array specify Mat data array to be written.
        .       @param dslabel specify the target hdf5 dataset label.
        .       @param dims_offset each array member specify the offset location
        .              over dataset's each dimensions from where InputArray will be (over)written into dataset.
        .       @param dims_counts each array member specify the amount of data over dataset's
        .              each dimensions from InputArray that will be written into dataset.
        .   
        .       Writes Mat object into targeted dataset and **autoexpand** dataset dimension if allowed.
        .   
        .       @note Unlike dswrite(), datasets are **not** created **automatically**. Only Mat is supported and it must be **continuous**.
        .       If dsinsert() happens over outer regions of dataset dimensions and on that dimension of dataset is in **unlimited** mode then
        .       dataset is expanded, otherwise exception is thrown. To create datasets with **unlimited** property on specific or more
        .       dimensions see dscreate() and the optional H5_UNLIMITED flag at creation time. It is not thread safe over same dataset
        .       but multiple datasets can be merged inside a single hdf5 file.
        .   
        .       - Example below creates **unlimited** rows x 100 cols and expands rows 5 times with dsinsert() using single 100x100 CV_64FC2
        .       over the dataset. Final size will have 5x100 rows and 100 cols, reflecting H matrix five times over row's span. Chunks size is
        .       100x100 just optimized against the H matrix size having compression disabled. If routine is called multiple times dataset will be
        .       just overwritten:
        .       @code{.cpp}
        .         // dual channel hilbert matrix
        .         cv::Mat H(50, 100, CV_64FC2);
        .         for(int i = 0; i < H.rows; i++)
        .           for(int j = 0; j < H.cols; j++)
        .           {
        .               H.at<cv::Vec2d>(i,j)[0] =  1./(i+j+1);
        .               H.at<cv::Vec2d>(i,j)[1] = -1./(i+j+1);
        .               count++;
        .           }
        .         // open / autocreate hdf5 file
        .         cv::Ptr<cv::hdf::HDF5> h5io = cv::hdf::open( "mytest.h5" );
        .         // optimise dataset by chunks
        .         int chunks[2] = { 100, 100 };
        .         // create Unlimited x 100 CV_64FC2 space
        .         h5io->dscreate( cv::hdf::HDF5::H5_UNLIMITED, 100, CV_64FC2, "hilbert", cv::hdf::HDF5::H5_NONE, chunks );
        .         // write into first half
        .         int offset[2] = { 0, 0 };
        .         for ( int t = 0; t < 5; t++ )
        .         {
        .           offset[0] += 100 * t;
        .           h5io->dsinsert( H, "hilbert", offset );
        .         }
        .         // release
        .         h5io->close();
        .       @endcode
        """
        pass

    def dsread(self, dslabel, Array=None): # real signature unknown; restored from __doc__
        """
        dsread(dslabel[, Array]) -> Array
        .   
        
        
        
        dsread(dslabel, dims_offset[, Array]) -> Array
        .   
        
        
        
        dsread(dslabel, dims_offset[, Array[, dims_counts]]) -> Array
        .   
        
        
        
        dsread(dslabel, dims_offset, dims_counts[, Array]) -> Array
        .   @brief Read specific dataset from hdf5 file into Mat object.
        .       @param Array Mat container where data reads will be returned.
        .       @param dslabel specify the source hdf5 dataset label.
        .       @param dims_offset each array member specify the offset location over
        .              each dimensions from where dataset starts to read into OutputArray.
        .       @param dims_counts each array member specify the amount over dataset's each
        .              dimensions of dataset to read into OutputArray.
        .   
        .       Reads out Mat object reflecting the stored dataset.
        .   
        .       @note If hdf5 file does not exist an exception will be thrown. Use hlexists() to check dataset presence.
        .       It is thread safe.
        .   
        .       - Example below reads a dataset:
        .       @code{.cpp}
        .         // open hdf5 file
        .         cv::Ptr<cv::hdf::HDF5> h5io = cv::hdf::open( "mytest.h5" );
        .         // blank Mat container
        .         cv::Mat H;
        .         // read hibert dataset
        .         h5io->read( H, "hilbert" );
        .         // release
        .         h5io->close();
        .       @endcode
        .   
        .       - Example below perform read of 3x5 submatrix from second row and third element.
        .       @code{.cpp}
        .         // open hdf5 file
        .         cv::Ptr<cv::hdf::HDF5> h5io = cv::hdf::open( "mytest.h5" );
        .         // blank Mat container
        .         cv::Mat H;
        .         int offset[2] = { 1, 2 };
        .         int counts[2] = { 3, 5 };
        .         // read hibert dataset
        .         h5io->read( H, "hilbert", offset, counts );
        .         // release
        .         h5io->close();
        .       @endcode
        """
        pass

    def dswrite(self, Array, dslabel): # real signature unknown; restored from __doc__
        """
        dswrite(Array, dslabel) -> None
        .   
        
        
        
        dswrite(Array, dslabel, dims_offset) -> None
        .   
        
        
        
        dswrite(Array, dslabel, dims_offset[, dims_counts]) -> None
        .   
        
        
        
        dswrite(Array, dslabel, dims_offset, dims_counts) -> None
        .   @brief Write or overwrite a Mat object into specified dataset of hdf5 file.
        .       @param Array specify Mat data array to be written.
        .       @param dslabel specify the target hdf5 dataset label.
        .       @param dims_offset each array member specify the offset location
        .              over dataset's each dimensions from where InputArray will be (over)written into dataset.
        .       @param dims_counts each array member specifies the amount of data over dataset's
        .              each dimensions from InputArray that will be written into dataset.
        .   
        .       Writes Mat object into targeted dataset.
        .   
        .       @note If dataset is not created and does not exist it will be created **automatically**. Only Mat is supported and
        .       it must be **continuous**. It is thread safe but it is recommended that writes to happen over separate non-overlapping
        .       regions. Multiple datasets can be written inside a single hdf5 file.
        .   
        .       - Example below writes a 100x100 CV_64FC2 matrix into a dataset. No dataset pre-creation required. If routine
        .       is called multiple times dataset will be just overwritten:
        .       @code{.cpp}
        .         // dual channel hilbert matrix
        .         cv::Mat H(100, 100, CV_64FC2);
        .         for(int i = 0; i < H.rows; i++)
        .           for(int j = 0; j < H.cols; j++)
        .           {
        .               H.at<cv::Vec2d>(i,j)[0] =  1./(i+j+1);
        .               H.at<cv::Vec2d>(i,j)[1] = -1./(i+j+1);
        .               count++;
        .           }
        .         // open / autocreate hdf5 file
        .         cv::Ptr<cv::hdf::HDF5> h5io = cv::hdf::open( "mytest.h5" );
        .         // write / overwrite dataset
        .         h5io->dswrite( H, "hilbert" );
        .         // release
        .         h5io->close();
        .       @endcode
        .   
        .       - Example below writes a smaller 50x100 matrix into 100x100 compressed space optimised by two 50x100 chunks.
        .       Matrix is written twice into first half (0->50) and second half (50->100) of data space using offset.
        .       @code{.cpp}
        .         // dual channel hilbert matrix
        .         cv::Mat H(50, 100, CV_64FC2);
        .         for(int i = 0; i < H.rows; i++)
        .           for(int j = 0; j < H.cols; j++)
        .           {
        .               H.at<cv::Vec2d>(i,j)[0] =  1./(i+j+1);
        .               H.at<cv::Vec2d>(i,j)[1] = -1./(i+j+1);
        .               count++;
        .           }
        .         // open / autocreate hdf5 file
        .         cv::Ptr<cv::hdf::HDF5> h5io = cv::hdf::open( "mytest.h5" );
        .         // optimise dataset by two chunks
        .         int chunks[2] = { 50, 100 };
        .         // create 100x100 CV_64FC2 compressed space
        .         h5io->dscreate( 100, 100, CV_64FC2, "hilbert", 9, chunks );
        .         // write into first half
        .         int offset1[2] = { 0, 0 };
        .         h5io->dswrite( H, "hilbert", offset1 );
        .         // write into second half
        .         int offset2[2] = { 50, 0 };
        .         h5io->dswrite( H, "hilbert", offset2 );
        .         // release
        .         h5io->close();
        .       @endcode
        """
        pass

    def grcreate(self, grlabel): # real signature unknown; restored from __doc__
        """
        grcreate(grlabel) -> None
        .   @brief Create a group.
        .       @param grlabel specify the hdf5 group label.
        .   
        .       Create a hdf5 group with default properties. The group is closed automatically after creation.
        .   
        .       @note Groups are useful for better organising multiple datasets. It is possible to create subgroups within any group.
        .       Existence of a particular group can be checked using hlexists(). In case of subgroups, a label would be e.g: 'Group1/SubGroup1'
        .       where SubGroup1 is within the root group Group1. Before creating a subgroup, its parent group MUST be created.
        .   
        .       - In this example, Group1 will have one subgroup called SubGroup1:
        .   
        .        @snippet samples/create_groups.cpp create_group
        .   
        .        The corresponding result visualized using the HDFView tool is
        .   
        .        ![Visualization of groups using the HDFView tool](pics/create_groups.png)
        .   
        .       @note When a dataset is created with dscreate() or kpcreate(), it can be created within a group by specifying the
        .       full path within the label. In our example, it would be: 'Group1/SubGroup1/MyDataSet'. It is not thread safe.
        """
        pass

    def hlexists(self, label): # real signature unknown; restored from __doc__
        """
        hlexists(label) -> retval
        .   @brief Check if label exists or not.
        .       @param label specify the hdf5 dataset label.
        .   
        .       Returns **true** if dataset exists, and **false** otherwise.
        .   
        .       @note Checks if dataset, group or other object type (hdf5 link) exists under the label name. It is thread safe.
        """
        pass

    def kpgetsize(self, kplabel, dims_flag=None): # real signature unknown; restored from __doc__
        """
        kpgetsize(kplabel[, dims_flag]) -> retval
        .   @brief Fetch keypoint dataset size
        .       @param kplabel specify the hdf5 dataset label to be measured.
        .       @param dims_flag will fetch dataset dimensions on H5_GETDIMS, and dataset maximum dimensions on H5_GETMAXDIMS.
        .   
        .       Returns size of keypoints dataset.
        .   
        .       @note Resulting size will match the amount of keypoints. By default H5_GETDIMS will return actual dataset dimension.
        .       Using H5_GETMAXDIM flag will get maximum allowed dimension which normally match actual dataset dimension but can hold
        .       H5_UNLIMITED value if dataset was prepared in **unlimited** mode. It can be useful to check existing dataset dimension
        .       before overwrite it as whole or subset. Trying to write with oversized source data into dataset target will thrown
        .       exception. The H5_GETCHUNKDIMS will return the dimension of chunk if dataset was created with chunking options otherwise
        .       returned vector size will be zero.
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


