#include "SampleFilter.hpp"

#include <algorithm>


namespace{





  class samplePointF{
  private:
    std::vector<double> d;

    std::vector<double> tdu;
    std::vector<double> tdv;

    std::vector<double> tcu;
    std::vector<double> tcv;

    std::vector<double> px;
    std::vector<double> py;
    std::vector<double> pz;

  public:
    samplePointF()
      : d(),
	tdu(),
	tdv(),
	tcu(),
	tcv(),
	px(),
	py(),
	pz()
    {}

    ~samplePointF()
    {}

	
    void add(const samplePoint& s){
      d.push_back(s.depth);

      tdu.push_back(s.tex_depth.u);
      tdv.push_back(s.tex_depth.v);

      tcu.push_back(s.tex_color.u);
      tcv.push_back(s.tex_color.v);

      px.push_back(s.pos_real[0]);
      py.push_back(s.pos_real[1]);
      pz.push_back(s.pos_real[2]);

    }
#if 0
    void norm(){
      d /= n;

      tdu /= n;
      tdv /= n;

      tcu /= n;
      tcv /= n;

      px /= n;
      py /= n;
      pz /= n;
    }
#endif

    double filter(const std::vector<double>& tmp){

      const float range = 2.0;

      
      const double sum = std::accumulate(tmp.begin(), tmp.end(), 0.0);
      const double mean = sum / tmp.size();
      
      const double sq_sum = std::inner_product(tmp.begin(), tmp.end(), tmp.begin(), 0.0);
      const double stdev = std::sqrt(sq_sum / tmp.size() - mean * mean);
      

      std::vector<double> filt;
      const double minf = mean - range * stdev;
      const double maxf = mean + range * stdev;
      for(unsigned i = 0; i < tmp.size(); ++i){
	if(tmp[i] > minf && tmp[i] < maxf){
	  filt.push_back(tmp[i]);
	}
      }

      const double sumf = std::accumulate(filt.begin(), filt.end(), 0.0);
      const double meanf = sumf / filt.size();

      std::cout << "filtering over: " << tmp.size() << "frames. mean | stdev before: " << mean << " | " << stdev << " mean filtered: " << meanf << " outliers removed " << tmp.size() - filt.size() << std::endl;

      if(filt.size()){
	return meanf;
      }
      else{
	return mean;
      }
    }

    samplePoint sp(){
      samplePoint res;


      res.depth = filter(d);


      res.tex_depth.u = filter(tdu);
      res.tex_depth.v = filter(tdv);

      res.tex_color.u = filter(tcu);
      res.tex_color.v = filter(tcv);

      res.pos_real = glm::vec3(filter(px),filter(py),filter(pz));


      return res;
    }

  };


}



  SampleFilter::SampleFilter(unsigned bs)
    : m_bs(bs),
      m_s(){

    for(unsigned i = 0; i < m_bs; ++i){
      m_s.push_back(new std::vector<samplePoint>);
    }

  }


  SampleFilter::~SampleFilter()
  {}


  void
  SampleFilter::clear(){

    for(unsigned i = 0; i < m_bs; ++i){
      m_s[i]->clear();
    }


  }

  void
  SampleFilter::addSamples(std::vector<samplePoint>& board){

    for(unsigned i = 0; i < m_bs; ++i){
      m_s[i]->push_back(board[i]);
    }


  }

  std::vector<samplePoint>
  SampleFilter::getFiltered(){

    std::vector<samplePoint> res;

    // for each board average samplePoints

    for(unsigned i = 0; i < m_bs; ++i){
      

      samplePointF avg;
      const unsigned num(m_s[i]->size());
      for(unsigned s = 0; s < num; ++s){
	avg.add((*m_s[i])[s]);
      }
      std::cout << "corner point: " << i << " ----------------------------------------------------------------------------------" << std::endl;
      res.push_back(avg.sp());

    }


    return res;

  }


