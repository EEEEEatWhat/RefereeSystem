#ifndef REFEREE_SYSTEM_BASE_HPP
#define REFEREE_SYSTEM_BASE_HPP

namespace RM_referee
{
  class RegularPolygon
  {
    public:
      virtual void initialize(double side_length) = 0;
      virtual double area() = 0;
      virtual ~RegularPolygon(){}

    protected:
      RegularPolygon(){}
  };
}  // namespace polygon_base

#endif  // REFEREE_SYSTEM_BASE_HPP