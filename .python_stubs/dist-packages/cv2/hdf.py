# encoding: utf-8
# module cv2.hdf
# from /usr/lib/python3/dist-packages/cv2.cpython-38-x86_64-linux-gnu.so
# by generator 1.147
# no doc
# no imports

# Variables with simple values

__loader__ = None

__spec__ = None

# functions

def open(HDF5Filename): # real signature unknown; restored from __doc__
    """
    open(HDF5Filename) -> retval
    .   @brief Open or create hdf5 file
    .     @param HDF5Filename specify the HDF5 filename.
    .   
    .     Returns a pointer to the hdf5 object class
    .   
    .     @note If the specified file does not exist, it will be created using default properties.
    .     Otherwise, it is opened in read and write mode with default access properties.
    .     Any operations except dscreate() functions on object
    .     will be thread safe. Multiple datasets can be created inside a single hdf5 file, and can be accessed
    .     from the same hdf5 object from multiple instances as long read or write operations are done over
    .     non-overlapping regions of dataset. Single hdf5 file also can be opened by multiple instances,
    .     reads and writes can be instantiated at the same time as long as non-overlapping regions are involved. Object
    .     is released using close().
    .   
    .     - Example below opens and then releases the file.
    .     @code{.cpp}
    .       // open / auto create hdf5 file
    .       cv::Ptr<cv::hdf::HDF5> h5io = cv::hdf::open( "mytest.h5" );
    .       // ...
    .       // release
    .       h5io->close();
    .     @endcode
    .   
    .     ![Visualization of 10x10 CV_64FC2 (Hilbert matrix) using HDFView tool](pics/hdfview_demo.gif)
    .   
    .     - Text dump (3x3 Hilbert matrix) of hdf5 dataset using **h5dump** tool:
    .     @code{.txt}
    .     $ h5dump test.h5
    .     HDF5 "test.h5" {
    .     GROUP "/" {
    .        DATASET "hilbert" {
    .           DATATYPE  H5T_ARRAY { [2] H5T_IEEE_F64LE }
    .           DATASPACE  SIMPLE { ( 3, 3 ) / ( 3, 3 ) }
    .           DATA {
    .           (0,0): [ 1, -1 ], [ 0.5, -0.5 ], [ 0.333333, -0.333333 ],
    .           (1,0): [ 0.5, -0.5 ], [ 0.333333, -0.333333 ], [ 0.25, -0.25 ],
    .           (2,0): [ 0.333333, -0.333333 ], [ 0.25, -0.25 ], [ 0.2, -0.2 ]
    .           }
    .        }
    .     }
    .     }
    .     @endcode
    """
    pass

# no classes
