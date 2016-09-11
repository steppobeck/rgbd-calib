#include <ChessboardSampling.hpp>
#include <calibvolume.hpp>
#include <rgbdsensor.hpp>
#include <DataTypes.hpp>
#include <CMDParser.hpp>
#include <sweepsampler.hpp>
#include <calibrator.hpp>

#include <fstream>
#include <iostream>
#include <iomanip>


unsigned tracking_num_samples_taken = 0;
unsigned color_num_samples_taken = 0;

double sampleQuality(const std::string& filename_xyz, const std::string& filename_uv,
		     ChessboardSampling& cs_sweep,
		     const RGBDConfig& cfg,
		     const Checkerboard& cb,
		     const float tracking_offset_time,
		     const float color_offset_time,
		     const unsigned optimization_stride,
		     const unsigned idwneighbours,
		     bool using_nni,
		     const std::string& basefilename,
		     RGBDSensor* sensor = 0, const glm::mat4* eye_d_to_world = 0){
    CalibVolume cv_sweep(filename_xyz.c_str(), filename_uv.c_str());
    cs_sweep.loadChessboards();
    SweepSampler ss(&cb, &cv_sweep, &cfg);
    ss.extractSamples(&cs_sweep, tracking_offset_time, color_offset_time, optimization_stride);
    const std::vector<samplePoint>& sps = ss.getSamplePoints();
    Calibrator   c;
    c.using_nni = using_nni;
    c.applySamples(&cv_sweep, sps, cfg, idwneighbours, basefilename.c_str(), sensor, eye_d_to_world);
    ++tracking_num_samples_taken;
    return c.evaluatePlanes(&cv_sweep, &cs_sweep, cfg, optimization_stride);
}



double sampleQuality3D(const std::string& filename_xyz, const std::string& filename_uv,
		       ChessboardSampling& cs_sweep,
		       const RGBDConfig& cfg,
		       const Checkerboard& cb,
		       const float tracking_offset_time,
		       const float color_offset_time,
		       const unsigned optimization_stride,
		       const unsigned idwneighbours,
		       bool using_nni,
		       const std::string& basefilename,
		       RGBDSensor* sensor = 0, const glm::mat4* eye_d_to_world = 0){
  CalibVolume cv_sweep(filename_xyz.c_str(), filename_uv.c_str());
  cs_sweep.loadChessboards();
  SweepSampler ss(&cb, &cv_sweep, &cfg);
  ss.extractSamples(&cs_sweep, tracking_offset_time, color_offset_time, optimization_stride);
  const std::vector<samplePoint>& sps = ss.getSamplePoints();
  Calibrator   c;
  c.using_nni = using_nni;
  c.applySamples(&cv_sweep, sps, cfg, idwneighbours, basefilename.c_str(), sensor, eye_d_to_world);
  ++tracking_num_samples_taken;
  return c.evaluate3DError(&cv_sweep, &cs_sweep, &cb, cfg, tracking_offset_time, optimization_stride);
}

double sampleQuality2D(const std::string& filename_xyz, const std::string& filename_uv,
		       ChessboardSampling& cs_sweep,
		       const RGBDConfig& cfg,
		       const Checkerboard& cb,
		       const float tracking_offset_time,
		       const float color_offset_time,
		       const unsigned optimization_stride,
		       const unsigned idwneighbours,
		       bool using_nni,
		       const std::string& basefilename,
		       RGBDSensor* sensor = 0, const glm::mat4* eye_d_to_world = 0){
  CalibVolume cv_sweep(filename_xyz.c_str(), filename_uv.c_str());
  cs_sweep.loadChessboards();
  SweepSampler ss(&cb, &cv_sweep, &cfg);
  ss.extractSamples(&cs_sweep, tracking_offset_time, color_offset_time, optimization_stride);
  const std::vector<samplePoint>& sps = ss.getSamplePoints();
  Calibrator   c;
  c.using_nni = using_nni;
  c.applySamples(&cv_sweep, sps, cfg, idwneighbours, basefilename.c_str(), sensor, eye_d_to_world);
  ++color_num_samples_taken;
  return c.evaluate2DError(&cv_sweep, &cs_sweep, cfg, color_offset_time, optimization_stride);
}


double sampleGradient(const std::string& filename_xyz, const std::string& filename_uv,
		      ChessboardSampling& cs_sweep,
		      const RGBDConfig& cfg,
		      const Checkerboard& cb,
		      const double left_tracking_offset_time,
		      const double right_tracking_offset_time,
		      const float color_offset_time,
		      const unsigned optimization_stride,
		      const unsigned idwneighbours,
		      bool using_nni,
		      const std::string& basefilename,
		      RGBDSensor* sensor = 0, const glm::mat4* eye_d_to_world = 0){

  const double  y_left = sampleQuality(filename_xyz, filename_uv, cs_sweep, cfg, cb,
				       left_tracking_offset_time, color_offset_time, optimization_stride, idwneighbours, using_nni, basefilename,
				       sensor, eye_d_to_world);

  const double  y_right = sampleQuality(filename_xyz, filename_uv, cs_sweep, cfg, cb,
					right_tracking_offset_time, color_offset_time, optimization_stride, idwneighbours, using_nni, basefilename,
					sensor, eye_d_to_world);
  
  return (y_right - y_left)/(right_tracking_offset_time - left_tracking_offset_time);

}


double sampleGradient3D(const std::string& filename_xyz, const std::string& filename_uv,
			ChessboardSampling& cs_sweep,
			const RGBDConfig& cfg,
			const Checkerboard& cb,
			const double left_tracking_offset_time,
			const double right_tracking_offset_time,
			const float color_offset_time,
			const unsigned optimization_stride,
			const unsigned idwneighbours,
			bool using_nni,
			const std::string& basefilename,
			RGBDSensor* sensor = 0, const glm::mat4* eye_d_to_world = 0){

  const double  y_left = sampleQuality3D(filename_xyz, filename_uv, cs_sweep, cfg, cb,
					 left_tracking_offset_time, color_offset_time, optimization_stride, idwneighbours, using_nni, basefilename,
					 sensor, eye_d_to_world);

  const double  y_right = sampleQuality3D(filename_xyz, filename_uv, cs_sweep, cfg, cb,
					  right_tracking_offset_time, color_offset_time, optimization_stride, idwneighbours, using_nni, basefilename,
					  sensor, eye_d_to_world);
  
  return (y_right - y_left)/(right_tracking_offset_time - left_tracking_offset_time);

}


double sampleGradient2D(const std::string& filename_xyz, const std::string& filename_uv,
			ChessboardSampling& cs_sweep,
			const RGBDConfig& cfg,
			const Checkerboard& cb,
			const double left_color_offset_time,
			const double right_color_offset_time,
			const float tracking_offset_time,
			const unsigned optimization_stride,
			const unsigned idwneighbours,
			bool using_nni,
			const std::string& basefilename,
			RGBDSensor* sensor = 0, const glm::mat4* eye_d_to_world = 0){

  const double  y_left = sampleQuality2D(filename_xyz, filename_uv, cs_sweep, cfg, cb,
					 tracking_offset_time, left_color_offset_time, optimization_stride, idwneighbours, using_nni, basefilename,
					 sensor, eye_d_to_world);

  const double  y_right = sampleQuality2D(filename_xyz, filename_uv, cs_sweep, cfg, cb,
					  tracking_offset_time, right_color_offset_time, optimization_stride, idwneighbours, using_nni, basefilename,
					  sensor, eye_d_to_world);
  
  return (y_right - y_left)/(right_color_offset_time - left_color_offset_time);

}

float refine(const std::string& filename_xyz, const std::string& filename_uv,
	     ChessboardSampling& cs_sweep,
	     const RGBDConfig& cfg,
	     const Checkerboard& cb,
	     const float best_tracking_offset_time,
	     const float color_offset_time,
	     const unsigned optimization_stride,
	     const unsigned idwneighbours,
	     std::ofstream* logfile,
	     bool using_nni,
	     const std::string& basefilename,
	     RGBDSensor* sensor = 0, const glm::mat4* eye_d_to_world = 0){





  const double min_gradient  = 0.000005;
  const double gradient_step = 0.03;
  const double step = 1.0;
  float tracking_offset_time = best_tracking_offset_time;

  for(unsigned rs = 0; rs < 15; ++rs){

    double gradient = sampleGradient(filename_xyz, filename_uv, cs_sweep, cfg, cb,
				     tracking_offset_time - 0.5 * gradient_step,
				     tracking_offset_time + 0.5 * gradient_step,
				     color_offset_time, optimization_stride, idwneighbours, using_nni, basefilename,
				     sensor, eye_d_to_world);

    float avg_quality = sampleQuality(filename_xyz, filename_uv, cs_sweep, cfg, cb,
				      tracking_offset_time, color_offset_time,
				      optimization_stride, idwneighbours, using_nni, basefilename,
				      sensor, eye_d_to_world);

    std::cout << "INFO: " << rs << " refine: at " << std::setprecision(10)
	      << tracking_offset_time << " avg_quality: " << avg_quality << " gradient: " << gradient << std::endl;

    if(logfile != 0){
      *logfile << tracking_offset_time * 1000 << ", " << (avg_quality-0.999)*1000.0 << std::endl;
    }

    if(std::abs(gradient) > min_gradient){
      tracking_offset_time = tracking_offset_time + step * gradient;
    }
    else{
      break;
    }
  }
  
  std::cout << "INFO: refine: finishing refinement at " << std::setprecision(10) << tracking_offset_time << std::endl;

  if(logfile != 0){
    *logfile << tracking_offset_time * 1000 << ", 0.0" << std::endl;
  }

  return tracking_offset_time;
}


float refine3D(const std::string& filename_xyz, const std::string& filename_uv,
	       ChessboardSampling& cs_sweep,
	       const RGBDConfig& cfg,
	       const Checkerboard& cb,
	       const float start_tracking_offset_time,
	       const float color_offset_time,
	       const unsigned optimization_stride,
	       const unsigned idwneighbours,
	       std::ofstream* logfile,
	       bool using_nni,
	       const std::string& basefilename,
	       RGBDSensor* sensor = 0, const glm::mat4* eye_d_to_world = 0){





  const double min_gradient  = 0.000001;
  const double gradient_step = 0.01;
  const double step = -0.1;
  float tracking_offset_time = start_tracking_offset_time;
  
  for(unsigned rs = 0; rs < 15; ++rs){

    double gradient = sampleGradient3D(filename_xyz, filename_uv, cs_sweep, cfg, cb,
				       tracking_offset_time - 0.5 * gradient_step,
				       tracking_offset_time + 0.5 * gradient_step,
				       color_offset_time, optimization_stride, idwneighbours, using_nni, basefilename,
				       sensor, eye_d_to_world);


    float avg_3D_error = sampleQuality3D(filename_xyz, filename_uv, cs_sweep, cfg, cb,
					 tracking_offset_time, color_offset_time,
					 optimization_stride, idwneighbours, using_nni, basefilename,
					 sensor, eye_d_to_world);

    std::cout << "INFO: " << rs << " refine3D: at " << std::setprecision(10)
	      << tracking_offset_time << " avg_3D_error: " << avg_3D_error << " gradient: " << gradient << std::endl;



    if(logfile != 0){
      *logfile << tracking_offset_time * 1000 << ", " << avg_3D_error * 100.0 << std::endl;
    }

    if(std::abs(gradient) > min_gradient){
      tracking_offset_time = tracking_offset_time + step * gradient;
    }
    else{
      break;
    }
  }
  
  std::cout << "INFO: refine3D: finishing refinement at " << std::setprecision(10) << tracking_offset_time << std::endl;

  if(logfile != 0){
    *logfile << tracking_offset_time * 1000 << ", 0.0" << std::endl;
  }

  return tracking_offset_time;
}

float refine2D(const std::string& filename_xyz, const std::string& filename_uv,
	       ChessboardSampling& cs_sweep,
	       const RGBDConfig& cfg,
	       const Checkerboard& cb,
	       const float tracking_offset_time,
	       const float start_color_offset_time,
	       const unsigned optimization_stride,
	       const unsigned idwneighbours,
	       std::ofstream* logfile,
	       bool using_nni,
	       const std::string& basefilename,
	       RGBDSensor* sensor = 0, const glm::mat4* eye_d_to_world = 0){





  const double min_gradient  = 0.001;
  const double gradient_step = 0.005;
  const double step = -0.0002;
  float color_offset_time = start_color_offset_time;
  
  for(unsigned rs = 0; rs < 15; ++rs){

    double gradient = sampleGradient2D(filename_xyz, filename_uv, cs_sweep, cfg, cb,
				       color_offset_time - 0.5 * gradient_step,
				       color_offset_time + 0.5 * gradient_step,
				       tracking_offset_time, optimization_stride, idwneighbours, using_nni, basefilename,
				       sensor, eye_d_to_world);


    float avg_2D_error = sampleQuality2D(filename_xyz, filename_uv, cs_sweep, cfg, cb,
					 tracking_offset_time, color_offset_time,
					 optimization_stride, idwneighbours, using_nni, basefilename,
					 sensor, eye_d_to_world);

    std::cout << "INFO: " << rs << " refine2D: at " << std::setprecision(10)
	      << color_offset_time * 1000 << " avg_2D_error: " << avg_2D_error << " gradient: " << gradient << std::endl;



    if(logfile != 0){
      *logfile << color_offset_time * 1000 << ", " << avg_2D_error << std::endl;
    }

    if(std::abs(gradient) > min_gradient){
      color_offset_time = color_offset_time + step * gradient;
    }
    else{
      break;
    }
  }
  
  std::cout << "INFO: refine2D: finishing refinement at " << std::setprecision(10) << color_offset_time << std::endl;

  if(logfile != 0){
    *logfile << color_offset_time * 1000 << ", 0.0" << std::endl;
  }

  return color_offset_time;
}



void printParabolaToLog(const float x_min, const float x_max, const float x_step,
			const double x1, const double x2, const double x3,
			const double y1, const double y2, const double y3,
			std::ofstream* logfile){


  const double a = (x1*(y2-y3)+x2*(y3-y1)+x3*(y1-y2))/((x1-x2)*(x1-x3)*(x3-x2));
  const double b = (x1*x1*(y2-y3)+x2*x2*(y3-y1)+x3*x3*(y1-y2))/((x1-x2)*(x1-x3)*(x2-x3));
  const double c = (x1*x1*(x2*y3-x3*y2)+x1*(x3*x3*y2-x2*x2*y3)+x2*x3*y1*(x2-x3))/((x1-x2)*(x1-x3)*(x2-x3));

  // x;y;best_x;best_y
  for(float x = x_min; x < x_max; x += x_step){
    const float y = a*(x*x) + b*x + c;
    *logfile << x * 1000 << ", " << y << std::endl;
  }
}
// from: http://www.arndt-bruenner.de/mathe/10/parabeldurchdreipunkte.htm
float optimizeParabelFitting(const std::string& filename_xyz, const std::string& filename_uv,
			     ChessboardSampling& cs_sweep,
			     const RGBDConfig& cfg,
			     const Checkerboard& cb,
			     const float tracking_offset_time_min,
			     const float tracking_offset_time_max,
			     const float color_offset_time,
			     const unsigned optimization_stride,
			     const unsigned idwneighbours,
			     std::ofstream* logfile,
			     bool using_nni,
			     const std::string& basefilename,
			     RGBDSensor* sensor = 0, const glm::mat4* eye_d_to_world = 0){



  const double x1 = tracking_offset_time_min;
  const double x2 = 0.5f*(tracking_offset_time_min + tracking_offset_time_max);
  const double x3 = tracking_offset_time_max;

  const double d  = x2 - x1;

  const double  y1 = sampleQuality(filename_xyz, filename_uv, cs_sweep, cfg, cb,
				   x1, color_offset_time, optimization_stride, idwneighbours, using_nni, basefilename,
				   sensor, eye_d_to_world);

  const double  y2 = sampleQuality(filename_xyz, filename_uv, cs_sweep, cfg, cb,
				   x2, color_offset_time, optimization_stride, idwneighbours, using_nni, basefilename,
				   sensor, eye_d_to_world);

  const double  y3 = sampleQuality(filename_xyz, filename_uv, cs_sweep, cfg, cb,
				   x3, color_offset_time, optimization_stride, idwneighbours, using_nni, basefilename,
				   sensor, eye_d_to_world);

  if(logfile != 0){ // (avg_quality - 0.999)*1000.0
    printParabolaToLog(x1, x3, 0.001,
		       x1, x2, x3,
		       (y1 - 0.999)*1000.0, (y2-0.999)*1000.0, (y3-0.999)*1000.0,
		       logfile);
  }

  const double xs = x2 + (0.5*d*(y3 - y1))/(2.0*y2 - y1 - y3);

  if(logfile != 0){
    *logfile << xs * 1000 << ", 0.0" << std::endl;
  }


  return float(xs);
}



float optimizeBruteForce(const std::string& filename_xyz, const std::string& filename_uv,
			 ChessboardSampling& cs_sweep,
			 const RGBDConfig& cfg,
			 const Checkerboard& cb,
			 const float tracking_offset_time_min,
			 const float tracking_offset_time_max,
			 const float tracking_offset_time_step,
			 const float color_offset_time,
			 const unsigned optimization_stride,
			 const unsigned idwneighbours,
			 std::ofstream* logfile,
			 bool using_nni,
			 const std::string& basefilename,
			 RGBDSensor* sensor = 0, const glm::mat4* eye_d_to_world = 0){

  



  float best_tracking_offset_time = (tracking_offset_time_max + tracking_offset_time_min) * 0.5f;
  double best_avg_quality = 0.0f;
  for(float tracking_offset_time = tracking_offset_time_min;
      tracking_offset_time < tracking_offset_time_max;
      tracking_offset_time += tracking_offset_time_step){
    
    const double avg_quality = sampleQuality(filename_xyz, filename_uv, cs_sweep, cfg, cb,
					     tracking_offset_time, color_offset_time,
					     optimization_stride, idwneighbours,
					     using_nni, basefilename,
					     sensor, eye_d_to_world);


    if(avg_quality > best_avg_quality){
      best_avg_quality = avg_quality;
      best_tracking_offset_time = tracking_offset_time;
    }

    if(logfile != 0){ // (avg_quality - 0.999)*1000.0
      *logfile << tracking_offset_time * 1000 << ", " << std::setprecision(10) << (avg_quality - 0.999)*1000.0 << std::endl;
    }

  }

  if(logfile != 0){
    *logfile << best_tracking_offset_time * 1000 << ", 0.0" << std::endl;
  }

  return best_tracking_offset_time;
}

float optimizeBruteForce3D(const std::string& filename_xyz, const std::string& filename_uv,
			   ChessboardSampling& cs_sweep,
			   const RGBDConfig& cfg,
			   const Checkerboard& cb,
			   const float tracking_offset_time_min,
			   const float tracking_offset_time_max,
			   const float tracking_offset_time_step,
			   const float color_offset_time,
			   const unsigned optimization_stride,
			   const unsigned idwneighbours,
			   std::ofstream* logfile,
			   bool using_nni,
			   const std::string& basefilename,
			   RGBDSensor* sensor = 0, const glm::mat4* eye_d_to_world = 0){

  


  float best_tracking_offset_time = (tracking_offset_time_max + tracking_offset_time_min) * 0.5f;
  double best_avg_3D_error = std::numeric_limits<double>::max();
  for(float tracking_offset_time = tracking_offset_time_min;
      tracking_offset_time < tracking_offset_time_max;
      tracking_offset_time += tracking_offset_time_step){
    
    const double avg_3D_error = sampleQuality3D(filename_xyz, filename_uv, cs_sweep, cfg, cb,
						tracking_offset_time, color_offset_time,
						optimization_stride, idwneighbours,
						using_nni, basefilename,
						sensor, eye_d_to_world);

    if(avg_3D_error < best_avg_3D_error){
      best_avg_3D_error = avg_3D_error;
      best_tracking_offset_time = tracking_offset_time;
    }

    if(logfile != 0){
      *logfile << tracking_offset_time * 1000 << ", " << std::setprecision(10) << avg_3D_error * 100.0 << std::endl;
    }
  }

  if(logfile != 0){
    *logfile << best_tracking_offset_time * 1000 << ", 0.0" << std::endl;
  }

  return best_tracking_offset_time;
}



float optimizeBruteForce2D(const std::string& filename_xyz, const std::string& filename_uv,
			   ChessboardSampling& cs_sweep,
			   const RGBDConfig& cfg,
			   const Checkerboard& cb,
			   const float color_offset_time_min,
			   const float color_offset_time_max,
			   const float color_offset_time_step,
			   const float tracking_offset_time,
			   const unsigned optimization_stride,
			   const unsigned idwneighbours,
			   std::ofstream* logfile,
			   bool using_nni,
			   const std::string& basefilename,
			   RGBDSensor* sensor = 0, const glm::mat4* eye_d_to_world = 0){

  


  float best_color_offset_time = (color_offset_time_max + color_offset_time_min) * 0.5f;
  double best_avg_2D_error = std::numeric_limits<double>::max();
  for(float color_offset_time = color_offset_time_min;
      color_offset_time < color_offset_time_max;
      color_offset_time += color_offset_time_step){
    
    const double avg_2D_error = sampleQuality2D(filename_xyz, filename_uv, cs_sweep, cfg, cb,
						tracking_offset_time, color_offset_time,
						optimization_stride, idwneighbours,
						using_nni, basefilename,
						sensor, eye_d_to_world);

    if(avg_2D_error < best_avg_2D_error){
      best_avg_2D_error = avg_2D_error;
      best_color_offset_time = color_offset_time;
    }

    if(logfile != 0){
      *logfile << color_offset_time * 1000 << ", " << std::setprecision(10) << avg_2D_error << std::endl;
    }

  }

  if(logfile != 0){
    *logfile << best_color_offset_time * 1000 << ", 0.0" << std::endl;
  }

  return best_color_offset_time;
}


float optimizeParabelFitting3D(const std::string& filename_xyz, const std::string& filename_uv,
			       ChessboardSampling& cs_sweep,
			       const RGBDConfig& cfg,
			       const Checkerboard& cb,
			       const float tracking_offset_time_min,
			       const float tracking_offset_time_max,
			       const float color_offset_time,
			       const unsigned optimization_stride,
			       const unsigned idwneighbours,
			       std::ofstream* logfile,
			       bool using_nni,
			       const std::string& basefilename,
			       RGBDSensor* sensor = 0, const glm::mat4* eye_d_to_world = 0){



  const double x1 = tracking_offset_time_min;
  const double x2 = 0.5f*(tracking_offset_time_min + tracking_offset_time_max);
  const double x3 = tracking_offset_time_max;

  const double d  = x2 - x1;

  const double  y1 = sampleQuality3D(filename_xyz, filename_uv, cs_sweep, cfg, cb,
				     x1, color_offset_time, optimization_stride, idwneighbours, using_nni, basefilename,
				     sensor, eye_d_to_world);

  const double  y2 = sampleQuality3D(filename_xyz, filename_uv, cs_sweep, cfg, cb,
				     x2, color_offset_time, optimization_stride, idwneighbours, using_nni, basefilename,
				     sensor, eye_d_to_world);

  const double  y3 = sampleQuality3D(filename_xyz, filename_uv, cs_sweep, cfg, cb,
				     x3, color_offset_time, optimization_stride, idwneighbours, using_nni, basefilename,
				     sensor, eye_d_to_world);

  if(logfile != 0){
    printParabolaToLog(x1, x3, 0.001,
		       x1, x2, x3,
		       y1 * 100.0, y2 * 100.0, y3 * 100.0,
		       logfile);
  }

  const double xs = x2 + (0.5*d*(y3 - y1))/(2.0*y2 - y1 - y3);

  if(logfile != 0){
    *logfile << xs * 1000 << ", 0.0" << std::endl;
  }

  return float(xs);
}



float optimizeParabelFitting2D(const std::string& filename_xyz, const std::string& filename_uv,
			       ChessboardSampling& cs_sweep,
			       const RGBDConfig& cfg,
			       const Checkerboard& cb,
			       const float color_offset_time_min,
			       const float color_offset_time_max,
			       const float tracking_offset_time,
			       const unsigned optimization_stride,
			       const unsigned idwneighbours,
			       std::ofstream* logfile,
			       bool using_nni,
			       const std::string& basefilename,
			       RGBDSensor* sensor = 0, const glm::mat4* eye_d_to_world = 0){



  const double x1 = color_offset_time_min;
  const double x2 = 0.5f*(color_offset_time_min + color_offset_time_max);
  const double x3 = color_offset_time_max;

  const double d  = x2 - x1;

  const double  y1 = sampleQuality2D(filename_xyz, filename_uv, cs_sweep, cfg, cb,
				     tracking_offset_time, x1, optimization_stride, idwneighbours, using_nni, basefilename,
				     sensor, eye_d_to_world);

  const double  y2 = sampleQuality2D(filename_xyz, filename_uv, cs_sweep, cfg, cb,
				     tracking_offset_time, x2, optimization_stride, idwneighbours, using_nni, basefilename,
				     sensor, eye_d_to_world);

  const double  y3 = sampleQuality2D(filename_xyz, filename_uv, cs_sweep, cfg, cb,
				     tracking_offset_time, x3, optimization_stride, idwneighbours, using_nni, basefilename,
				     sensor, eye_d_to_world);

  if(logfile != 0){
    printParabolaToLog(x1, x3, 0.001,
		       x1, x2, x3,
		       y1, y2, y3,
		       logfile);
  }

  const double xs = x2 + (0.5*d*(y3 - y1))/(2.0*y2 - y1 - y3);

  if(logfile != 0){
    *logfile << xs * 1000 << ", 0.0" << std::endl;
  }

  return float(xs);
}



int main(int argc, char* argv[]){

  std::string pose_offset_filename = "./poseoffset";
  unsigned cv_width  = 128;
  unsigned cv_height = 128;
  unsigned cv_depth  = 128;
  float    cv_min_d  = 0.5;
  float    cv_max_d  = 3.0;


  // ./sweep_calibrate ../../../data/23.cv ../../../data/23_init ../../../data/23_sweep -t -0.05 -0.0 -o 0 -l ../../../data/23.optimize_log.csv -i

  const float tracking_offset_time_step = 0.001; // in seconds, used for brute force optimization and for parabola debug sampling
  float tracking_offset_time_min = -0.05; // in seconds
  float tracking_offset_time_max = 0.0; // in seconds

  const float color_offset_time_step = 0.001; // in seconds, used for brute force optimization and for parabola debug sampling
  float color_offset_time_min = 0.0; // in seconds
  float color_offset_time_max = 0.02; // in seconds

  unsigned idwneighbours = 10;
  std::string logfilename = "";
  
  bool using_nni = false;
  const unsigned optimization_stride = 1;
  unsigned optimization_type = 0;
  bool append_samples = false;
  bool undistort = false;
  bool without_initial = false;
  bool full_optimization_evaluation = false;
  CMDParser p("calibvolumebasefilename checkerboardview_init checkerboardview_sweep samplesfilename");
  p.addOpt("p",1,"poseoffetfilename", "specify the filename of the poseoffset on disk, default: " + pose_offset_filename);
  p.addOpt("s",3,"size", "use this calibration volume size (width x height x depth), default: 128 128 256");
  p.addOpt("d",2,"depthrange", "use this depth range: 0.5 4.5");

  p.addOpt("t",2,"trackingoffsetrange", "min and max offset in seconds of the tracking system relative to depth frame of the sensor, e.g. -0.05 0.03, default: -0.5 0.0");
  p.addOpt("c",2,"coloroffsetrange", "min and max offset in seconds of the color frame relative to depth frame of the sensor, e.g. -0.05 0.0, default: -0.2 0.0");

  p.addOpt("n",1,"numneighbours", "the number of neighbours that should be used for IDW inverse distance weighting, default: 10");

  p.addOpt("o",1,"optimizationtype",
	   "(0): C.C. perform optimization using parabel fitting,\n\
 \t\t(1): C.C. brute force sampling,				 \n\
 \t\t(2): C.C. refine with gradient descent,			 \n\
 \t\t(4): 3D and 2D error is minimized using brute force sampling,	 \n\
 \t\t(5): 3D and 2D error is minimized using parabel fitting,	 \n\
 \t\t(6): 3D and 2D error refine with gradient descent,		 \n\
 \t\t(7): mid times are used,		 \n\
 \t\tdefault: 0");


  p.addOpt("l",1,"logfile", "log the optimization process to given filename, default: no logfile is used ");

  p.addOpt("i",-1,"nni", "do use natural neighbor interpolation if possible, default: false");

  p.addOpt("u", -1, "undistort", "enable undistortion of images before chessboardsampling, default: false");

  p.addOpt("w", -1, "without", "do not use initial calibration, default: false");

  p.addOpt("f", -1, "full", "perform full evaluation, default: false");

  p.init(argc,argv);

  if(p.getArgs().size() != 4){
    p.showHelp();
  }

  if(p.isOptSet("p")){
    pose_offset_filename = p.getOptsString("p")[0];
    std::cout << "setting poseoffetfilename to " << pose_offset_filename << std::endl;
  }


  if(p.isOptSet("s")){
    cv_width = p.getOptsInt("s")[0];
    cv_height = p.getOptsInt("s")[1];
    cv_depth = p.getOptsInt("s")[2];
  }

  if(p.isOptSet("d")){
    cv_min_d = p.getOptsInt("d")[0];
    cv_max_d = p.getOptsInt("d")[1];
  }


  if(p.isOptSet("t")){
    tracking_offset_time_min = p.getOptsFloat("t")[0];
    tracking_offset_time_max = p.getOptsFloat("t")[1];
  }

  if(p.isOptSet("c")){
    color_offset_time_min = p.getOptsFloat("c")[0];
    color_offset_time_max = p.getOptsFloat("c")[1];
  }

  if(p.isOptSet("n")){
    idwneighbours = p.getOptsInt("n")[0];
  }

  if(p.isOptSet("o")){
    optimization_type = p.getOptsInt("o")[0];
  }

  if(p.isOptSet("l")){
    logfilename = p.getOptsString("l")[0];
  }

  if(p.isOptSet("i")){
    using_nni = true;
  }
  if(p.isOptSet("u")){
    undistort = true;
  }
  if(p.isOptSet("w")){
    without_initial = true;
  }
  if(p.isOptSet("f")){
    full_optimization_evaluation = true;
    optimization_type = 0;
  }

  const std::string basefilename = p.getArgs()[0];
  std::string filename_xyz(basefilename + "_xyz");
  std::string filename_uv(basefilename + "_uv");
  const std::string filename_yml(basefilename + "_yml");
  CalibVolume cv_init(cv_width, cv_height, cv_depth, cv_min_d, cv_max_d);

  RGBDConfig cfg;
  cfg.read(filename_yml.c_str());

  RGBDSensor sensor(cfg);

  Checkerboard cb;
  cb.load_pose_offset(pose_offset_filename.c_str());
  // configure local 3D Points on chessboard here:
  for(unsigned y = 0; y < CB_HEIGHT; ++y){
    for(unsigned x = 0; x < CB_WIDTH; ++x){
      cb.points_local.push_back(glm::vec3(y * 0.075, x * 0.075,0.0));
    }
  }

  ChessboardSampling cbs(p.getArgs()[1].c_str(), cfg, undistort);
  cbs.init();

  const glm::mat4 eye_d_to_world = sensor.guess_eye_d_to_world_static(cbs, cb);
  std::cerr << "extrinsic of sensor is: " << eye_d_to_world << std::endl;
  std::cerr << "PLEASE note, the extrinsic guess can be improved by averaging" << std::endl;

  for(unsigned z = 0; z < cv_init.depth; ++z){
    for(unsigned y = 0; y < cv_init.height; ++y){
      for(unsigned x = 0; x < cv_init.width; ++x){

	const unsigned cv_index = (z * cv_init.width * cv_init.height) + (y * cv_init.width) + x;

	const float depth = (z + 0.5) * (cv_init.max_d - cv_init.min_d)/cv_init.depth + cv_init.min_d;
	const float xd = (x + 0.5) * sensor.config.size_d.x * 1.0/cv_init.width;
	const float yd = (y + 0.5) * sensor.config.size_d.y * 1.0/cv_init.height;

	glm::vec3 pos3D_local = sensor.calc_pos_d(xd, yd, depth);
	glm::vec2 pos2D_rgb   = sensor.calc_pos_rgb(pos3D_local);
	pos2D_rgb.x /= sensor.config.size_rgb.x;
	pos2D_rgb.y /= sensor.config.size_rgb.y;

	glm::vec4 pos3D_world = eye_d_to_world * glm::vec4(pos3D_local.x, pos3D_local.y, pos3D_local.z, 1.0);

	xyz pos3D;
	pos3D.x = pos3D_world.x;
	pos3D.y = pos3D_world.y;
	pos3D.z = pos3D_world.z;
	if(without_initial){
	  pos3D.x = 0.0;
	  pos3D.y = 0.0;
	  pos3D.z = 0.0;
	}
	cv_init.cv_xyz[cv_index] = pos3D;

	uv posUV;
	posUV.u = pos2D_rgb.x;
	posUV.v = pos2D_rgb.y;
	if(without_initial){
	  posUV.u = 0.0;
	  posUV.v = 0.0;
	}
	cv_init.cv_uv[cv_index] = posUV;
      }
    }
  }

  cv_init.save(filename_xyz.c_str(), filename_uv.c_str());
  
  // INITIAL CALIBRATION FINISHED HERE


  // 2. load recording from sweepfilename
  ChessboardSampling cs_sweep(p.getArgs()[2].c_str(), cfg, undistort);
  cs_sweep.init();


  // compute midtimes
  float best_tracking_offset_time = (tracking_offset_time_min + tracking_offset_time_max) * 0.5f;
  float best_color_offset_time    = (color_offset_time_min + color_offset_time_max) * 0.5f;

  std::ofstream* logfile = 0;
  switch(optimization_type){
  case 0:
    std::cout << "INFO: performing optimization using parabel fitting." << std::endl;
    if(logfilename != ""){
      logfile = new std::ofstream(std::string(logfilename + "_CP_CF.csv").c_str());
    }
    best_tracking_offset_time = optimizeParabelFitting(filename_xyz, filename_uv,
						       cs_sweep,
						       cfg,
						       cb,
						       tracking_offset_time_min,
						       tracking_offset_time_max,
						       best_color_offset_time,
						       optimization_stride,
						       idwneighbours,
						       logfile,
						       false/*using_nni*/,
						       basefilename,
						       &sensor, &eye_d_to_world);
    if(logfilename != ""){
      logfile->close();
      delete logfile;
    }  
    if(!full_optimization_evaluation){
      break;
    }
  case 1:
    std::cout << "INFO: performing optimization using brute force sampling." << std::endl;
    if(logfilename != ""){
      logfile = new std::ofstream(std::string(logfilename + "_CP_BF.csv").c_str());
    }
    best_tracking_offset_time = optimizeBruteForce(filename_xyz, filename_uv,
						   cs_sweep,
						   cfg,
						   cb,
						   tracking_offset_time_min,
						   tracking_offset_time_max,
						   tracking_offset_time_step,
						   best_color_offset_time,
						   optimization_stride,
						   idwneighbours,
						   logfile,
						   false/*using_nni*/,
						   basefilename,
						   &sensor, &eye_d_to_world);
    if(logfilename != ""){
      logfile->close();
      delete logfile;
    }

    if(!full_optimization_evaluation){
      break;
    }
#if 0
  case 2:
    std::cout << "INFO: performing optimization using refine with gradient descent." << std::endl;
    if(logfilename != ""){
      logfile = new std::ofstream(std::string(logfilename + "_CP_GD.csv").c_str());
    }
    best_tracking_offset_time = refine(filename_xyz, filename_uv,
				       cs_sweep,
				       cfg,
				       cb,
				       0.0/*starttime*/,
				       best_color_offset_time,
				       optimization_stride,
				       idwneighbours,
				       logfile,
				       false/*using_nni*/,
				       basefilename,
				       &sensor, &eye_d_to_world);
    if(logfilename != ""){
      logfile->close();
      delete logfile;
    }

    if(!full_optimization_evaluation){
      break;
    }
#endif
  case 4:
    std::cout << "INFO: performing optimization of 2D and 3D error using brute force sampling." << std::endl;
    if(logfilename != ""){
      logfile = new std::ofstream(std::string(logfilename + "_3D_BF.csv").c_str());
    }
    best_tracking_offset_time = optimizeBruteForce3D(filename_xyz, filename_uv,
						     cs_sweep,
						     cfg,
						     cb,
						     tracking_offset_time_min,
						     tracking_offset_time_max,
						     tracking_offset_time_step,
						     best_color_offset_time,
						     optimization_stride,
						     idwneighbours,
						     logfile,
						     false/*using_nni*/,
						     basefilename,
						     &sensor, &eye_d_to_world);
    if(logfilename != ""){
      logfile->close();
      delete logfile;
    }
    if(logfilename != ""){
      logfile = new std::ofstream(std::string(logfilename + "_2D_BF.csv").c_str());
    }
    best_color_offset_time = optimizeBruteForce2D(filename_xyz, filename_uv,
						  cs_sweep,
						  cfg,
						  cb,
						  color_offset_time_min,
						  color_offset_time_max,
						  color_offset_time_step,
						  best_tracking_offset_time,
						  optimization_stride,
						  idwneighbours,
						  logfile,
						  false/*using_nni*/,
						  basefilename,
						  &sensor, &eye_d_to_world);
    if(logfilename != ""){
      logfile->close();
      delete logfile;
    }
    if(!full_optimization_evaluation){
      break;
    }
  case 5:
    std::cout << "INFO: performing optimization of 2D and 3D error using parabel fitting." << std::endl;
    if(logfilename != ""){
      logfile = new std::ofstream(std::string(logfilename + "_3D_CF.csv").c_str());
    }
    best_tracking_offset_time = optimizeParabelFitting3D(filename_xyz, filename_uv,
							 cs_sweep,
							 cfg,
							 cb,
							 tracking_offset_time_min,
							 tracking_offset_time_max,
							 best_color_offset_time,
							 optimization_stride,
							 idwneighbours,
							 logfile,
							 false/*using_nni*/,
							 basefilename,
							 &sensor, &eye_d_to_world);
    if(logfilename != ""){
      logfile->close();
      delete logfile;
    }
    if(logfilename != ""){
      logfile = new std::ofstream(std::string(logfilename + "_2D_CF.csv").c_str());
    }
    best_color_offset_time = optimizeParabelFitting2D(filename_xyz, filename_uv,
						      cs_sweep,
						      cfg,
						      cb,
						      color_offset_time_min,
						      color_offset_time_max,
						      best_tracking_offset_time,
						      optimization_stride,
						      idwneighbours,
						      logfile,
						      false/*using_nni*/,
						      basefilename,
						      &sensor, &eye_d_to_world);
    if(logfilename != ""){
      logfile->close();
      delete logfile;
    }
    if(!full_optimization_evaluation){
      break;
    }
#if 0
  case 6:
    std::cout << "INFO: performing optimization using refine3D and refine2D with gradient descent." << std::endl;
    if(logfilename != ""){
      logfile = new std::ofstream(std::string(logfilename + "_3D_GD.csv").c_str());
    }
    best_tracking_offset_time = refine3D(filename_xyz, filename_uv,
					 cs_sweep,
					 cfg,
					 cb,
					 0.0/*starttime*/,
					 best_color_offset_time,
					 optimization_stride,
					 idwneighbours,
					 logfile,
					 false/*using_nni*/,
					 basefilename,
					 &sensor, &eye_d_to_world);
    if(logfilename != ""){
      logfile->close();
      delete logfile;
    }
    if(logfilename != ""){
      logfile = new std::ofstream(std::string(logfilename + "_2D_GD.csv").c_str());
    }
    best_color_offset_time = refine2D(filename_xyz, filename_uv,
				      cs_sweep,
				      cfg,
				      cb,
				      best_tracking_offset_time,
				      0.0/*starttime*/,
				      optimization_stride,
				      idwneighbours,
				      logfile,
				      false/*using_nni*/,
				      basefilename,
				      &sensor, &eye_d_to_world);
    if(logfilename != ""){
      logfile->close();
      delete logfile;
    }
    if(!full_optimization_evaluation){
      break;
    }
#endif
  case 7:
    std::cout << "INFO: performing no optimization, midtimes are used instead." << std::endl;
    if(!full_optimization_evaluation){
      break;
    }
  default:
    if(!full_optimization_evaluation){
      std::cerr << "ERROR: invalid optimization type: "
		<< optimization_type << " -> exiting...!" << std::endl;
      return -1;
    }
    break;
  }


  if(full_optimization_evaluation){
    return 0;
  }



  // do the calibration based on the best_tracking_offset_time
  cs_sweep.loadChessboards();
  SweepSampler ss(&cb, &cv_init, &cfg);
  ss.extractSamples(&cs_sweep, best_tracking_offset_time, best_color_offset_time);
  ss.appendSamplesToFile(p.getArgs()[3].c_str(), append_samples);
  
  const std::vector<samplePoint>& sps = ss.getSamplePoints();
  Calibrator   c;
  c.using_nni = using_nni;
  c.applySamples(&cv_init, sps, cfg, idwneighbours, basefilename.c_str(), &sensor, &eye_d_to_world);


  cv_init.save(filename_xyz.c_str(), filename_uv.c_str());


  std::cout << "INFO: !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
  std::cout << "INFO: tracking_num_samples_taken: " << tracking_num_samples_taken << std::endl;
  std::cout << "INFO: using best_tracking_offset_time: " << best_tracking_offset_time << std::endl;
  std::cout << "INFO: color_num_samples_taken: " << color_num_samples_taken << std::endl;
  std::cout << "INFO: using best_color_offset_time: " << best_color_offset_time << std::endl;
  std::cout << "INFO: !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
  
  return 0;
}
