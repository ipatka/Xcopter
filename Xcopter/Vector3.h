#ifndef __Xcopter__Vector3__
#define __Xcopter__Vector3__

#include <math.h>

class Vector3 {
public:
    Vector3(){
        x = y = z = 0;
    }
    
    Vector3(float x, float y, float z){
        this->x = x;
        this->y = y;
        this->z = z;
    }
    
    void set(float x, float y, float z){
        this->x = x;
        this->y = y;
        this->z = z;     
    }
    
    void set_rate(float x, float y, float z){
        this->x = x;
        this->y = y;
        this->z = z;
    }
    
    float length_squared(){
        return x*x + y*y + z*z;
    }
    
    float length(){
        return sqrt(length_squared());    
    }
    
    void normalise(){
        float len = length();
        x = x/len;
        y = y/len;
        z = z/len;
    }
    
    float dot_product(const Vector3 &v){
        return x*v.x + y*v.y + z*v.z;
    }
    
    Vector3 cross_product(const Vector3 &v){
        Vector3 temp(y*v.z - z*v.y, z*v.x - x*v.z, x*v.y - y*v.x);
        return temp;
    }
    
    //operator overides
    void operator ()(const float x, const float y, const float z) {
        this->x = x;
        this->y = y;
        this->z = z;
    }

    bool operator == (const Vector3 &v) {
        return (x==v.x && y==v.y && z==v.z);
    }

    bool operator != (const Vector3 &v) {
        return (x!=v.x || y!=v.y || z!=v.z);
    }

    const Vector3 &operator = (const Vector3 &v) {   
        x = v.x;
        y = v.y;
        z = v.z;     
        return *this;
    }
    
    const Vector3 operator - (void) const {
        return Vector3(-x,-y,-z);
    }
 
    const Vector3 operator + (const Vector3 &v) const {
        return Vector3(x+v.x, y+v.y, z+v.z);
    }  

    const Vector3 operator - (const Vector3 &v) const {
        return Vector3(x-v.x, y-v.y, z-v.z);
    }
    
    const Vector3 operator * (const float num) const {
        Vector3 temp;
        temp.x = x * num;
        temp.y = y * num;  
        temp.z = z * num;  
        return temp;
    }

    const Vector3 operator / (const float num) const {
        Vector3 temp;
        temp.x = x / num;
        temp.y = y / num;  
        temp.z = z / num;  
        return temp;
    }
    
    float operator * (const Vector3 &v) const {
        return x*v.x + y*v.y + z*v.z;
    }
    
    float x, y, z;
       
};

#endif