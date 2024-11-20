
// Code from MHRTF project - to be adapted into class
// using Vertex = std::array<float, 3>;
// std::array<float, 4> Plane_3Points(Vertex &A, Vertex &B, Vertex &C){
    
//     std::array<float, 4> Plane;
    
//     enum Params {a,b,c,d};
    
//     Plane[a]=(B[1]-A[1])*(C[2]-A[2])-(C[1]-A[1])*(B[2]-A[2]);
//     Plane[b]=(B[2]-A[2])*(C[0]-A[0])-(C[2]-A[2])*(B[0]-A[0]);
//     Plane[c]=(B[0]-A[0])*(C[1]-A[1])-(C[0]-A[0])*(B[1]-A[1]);
//     Plane[d]=-(Plane[a]*A[0]+Plane[b]*A[1]+Plane[c]*A[2]);
    
//     return Plane;
// }