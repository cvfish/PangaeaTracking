// Copyright (c) 2012 libmv authors.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to
// deal in the Software without restriction, including without limitation the
// rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
// sell copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
// IN THE SOFTWARE.
//
// Author: mierle@google.com (Keir Mierle)
//

#ifndef SAMPLE_H_
#define SAMPLE_H_

#include "main_engine/utils/global.h"

#include "opencv2/imgproc/imgproc.hpp"
#include "ceres/ceres.h"
#include "jet_extras.h"


// Sample the image at position (x, y) but use the gradient to
// propagate derivatives from x and y. This is needed to integrate the numeric
// image gradients with Ceres's autodiff framework.
template< typename T, class TImage >
T SampleWithDerivative(const TImage & intensityImage,
                       const TImage & intensityGradientX,
                       const TImage & intensityGradientY,
                       const T & x,
                       const T & y)
{
  typedef TImage ImageType;
  typedef typename ImageType::value_type PixelType;

  PixelType scalar_x = ceres::JetOps<T>::GetScalar(x);
  PixelType scalar_y = ceres::JetOps<T>::GetScalar(y);

  PixelType sample[3];
  // Sample intensity image and gradients
  SampleLinear( intensityImage, intensityGradientX, intensityGradientY,
                scalar_y, scalar_x, sample );
  T xy[2] = { x, y };
  return ceres::Chain< PixelType, 2, T >::Rule( sample[0], sample + 1, xy );
}

#endif  // SAMPLE_H_
