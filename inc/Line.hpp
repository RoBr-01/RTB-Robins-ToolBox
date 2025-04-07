#ifndef Line_HPP
#define Line_HPP

namespace RTB {

// A Line is basically a generalisation of a Ray. A Ray can (in theory) only
// travel forwards (positive values of t), whereas a Line is continuous and can
// travel in both forwards and backwards.

class Line {
   private:
    /* data */
   public:
    Line(/* args */);
    ~Line();
};

Line::Line(/* args */) {}

Line::~Line() {}

#endif  // Line_HPP
}  // RTB