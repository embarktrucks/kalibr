#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <aslam/cameras/GridCalibrationTargetGeneral.hpp>

namespace aslam {
namespace cameras {

/// \brief Construct a calibration target
GridCalibrationTargetGeneral::GridCalibrationTargetGeneral(size_t rows, size_t cols)
    : GridCalibrationTargetBase(rows, cols) {

  // allocate memory for the grid points
  _points.resize(size(), 3);
}

/// \brief initialize the object
void GridCalibrationTargetGeneral::initialize() {
  SM_ASSERT_TRUE(Exception, false, "GridCalibrationTargetGeneral::initialize function called. This method shouldn't be used for GridCalibrationTargetGeneral class");
}

/// \brief initialize a checkerboard grid (cols*rows = (cols)*(rows) internal grid points)
void GridCalibrationTargetGeneral::createGridPoints() {
  SM_ASSERT_TRUE(Exception, false, "GridCalibrationTargetGeneral::createGridPoints function called. This method shouldn't be used for GridCalibrationTargetGeneral class");
}

void GridCalibrationTargetGeneral::setPoints(const Eigen::MatrixXd &gridpoints2d){
    for (unsigned int r = 0; r < _rows; r++)
      for (unsigned int c = 0; c < _cols; c++)
        _points.row(gridCoordinatesToPoint(r, c)) = Eigen::Matrix<double, 1, 3>(gridpoints2d(gridCoordinatesToPoint(r, c),0), 
                                                                                gridpoints2d(gridCoordinatesToPoint(r, c),1), 0.0);
}

/// \brief extract the calibration target points from an image and write to an observation
bool GridCalibrationTargetGeneral::computeObservation(const cv::Mat & image,
           Eigen::MatrixXd & outImagePoints, std::vector<bool> &outCornerObserved) const {
    SM_ASSERT_TRUE(Exception, false, "GridCalibrationTargetGeneral::computeObservation function called. This method shouldn't be used for GridCalibrationTargetGeneral class");
}

}  // namespace cameras
}  // namespace aslam

//export explicit instantions for all included archives
#include <sm/boost/serialization.hpp>
#include <boost/serialization/export.hpp>
BOOST_CLASS_EXPORT_IMPLEMENT(aslam::cameras::GridCalibrationTargetGeneral);
