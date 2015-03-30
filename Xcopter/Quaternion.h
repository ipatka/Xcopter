#ifndef __Xcopter__Quaternion__
#define __Xcopter__Quaternion__

#include "Vector3.h"

class Quaternion {
public:
    Quaternion() {
        w = 0;    
    }
    Quaternion( float _w, float _x, float _y, float _z) {
        w = _w;
        v.set(_x,_y,_z);
    }
    Quaternion( float _w, Vector3 _v) {
        w = _w;
        v = _v;
    }
    Quaternion(float theta_x, float theta_y, float theta_z)
    {
        float cos_z_2 = cosf(0.5f*theta_z);
        float cos_y_2 = cosf(0.5f*theta_y);
        float cos_x_2 = cosf(0.5f*theta_x);

        float sin_z_2 = sinf(0.5f*theta_z);
        float sin_y_2 = sinf(0.5f*theta_y);
        float sin_x_2 = sinf(0.5f*theta_x);

        // and now compute quaternion
        w   = cos_z_2*cos_y_2*cos_x_2 + sin_z_2*sin_y_2*sin_x_2;
        v.x = cos_z_2*cos_y_2*sin_x_2 - sin_z_2*sin_y_2*cos_x_2;
        v.y = cos_z_2*sin_y_2*cos_x_2 + sin_z_2*cos_y_2*sin_x_2;
        v.z = sin_z_2*cos_y_2*cos_x_2 - cos_z_2*sin_y_2*sin_x_2;
    }
    ~Quaternion(){}
    
    void encode(char *buffer){
        int value = (w * (1 << 30));
        char* bytes = (char*)&value;
        for(int i = 0; i < 4; i ++){
            buffer[i] = bytes[3-i];
        }
        
        value = v.x * (1 << 30);
        for(int i = 0; i < 4; i ++){
            buffer[i+4] = bytes[3-i];
        }
        
        value = v.y * (1 << 30);
        for(int i = 0; i < 4; i ++){
            buffer[i+8] = bytes[3-i];
        }
        
        value = v.z  * (1 << 30);
        for(int i = 0; i < 4; i ++){
            buffer[i+12] = bytes[3-i];
        }            
    }
    
    void decode(const char *buffer){
        set((float)((((int32_t)buffer[0] << 24) + ((int32_t)buffer[1] << 16) + ((int32_t)buffer[2] << 8) + buffer[3]))* (1.0 / (1<<30)),
                (float)((((int32_t)buffer[4] << 24) + ((int32_t)buffer[5] << 16) + ((int32_t)buffer[6] << 8) + buffer[7]))* (1.0 / (1<<30)),
                (float)((((int32_t)buffer[8] << 24) + ((int32_t)buffer[9] << 16) + ((int32_t)buffer[10] << 8) + buffer[11]))* (1.0 / (1<<30)),
                (float)((((int32_t)buffer[12] << 24) + ((int32_t)buffer[13] << 16) + ((int32_t)buffer[14] << 8) + buffer[15]))* (1.0 / (1<<30)));    
    }
    
    void set( float _w, float _x, float _y, float _z) {
        w = _w;
        v.set(_x, _y, _z); 
    }

    void decode_rate(const char *buffer){
        set_rate((float)((((int32_t)buffer[16] << 24) + ((int32_t)buffer[17] << 16) + ((int32_t)buffer[18] << 8) + buffer[19]))* (1.0 / (1<<30)),
                (float)((((int32_t)buffer[20] << 24) + ((int32_t)buffer[21] << 16) + ((int32_t)buffer[22] << 8) + buffer[23]))* (1.0 / (1<<30)),
                (float)((((int32_t)buffer[24] << 24) + ((int32_t)buffer[25] << 16) + ((int32_t)buffer[26] << 8) + buffer[27]))* (1.0 / (1<<30)));   
    }

    void set_rate(float _x, float _y, float _z) {
        r.set_rate(_x, _y, _z);
    }


    float lengthSquared() const{
        return w * w + (v * v);  
    }
    
    float length() const{
        return sqrt(lengthSquared());
    }
    
    Quaternion normalise() const{       
        return (*this)/length();
    }
   
    Quaternion conjugate() const{       
        return Quaternion(w, -v);
    }
    
    Quaternion inverse() const {
        return conjugate() / lengthSquared();
    }
    
    float dot_product(const Quaternion &q){
        return q.v * v + q.w*w;       
    }
    
    Vector3 rotate(const Vector3 &v){
        return ((*this) *  Quaternion(0, v) * conjugate()).v;    
    }
    
    Quaternion lerp(const Quaternion &q2, float t) {
        if(t>1.0f) {
            t=1.0f;
        } else if(t < 0.0f){
            t=0.0f;
        }
        return ((*this)*(1-t) + q2*t).normalise();
    }
    
    Quaternion slerp( const Quaternion &q2, float t){
        if(t>1.0f) {
            t=1.0f;
        } else if(t < 0.0f){
            t=0.0f;
        }
        
        Quaternion q3;
        float dot = dot_product(q2);

        if (dot < 0)
        {
            dot = -dot;
            q3 = -q2;
        } else q3 = q2;
        
        if (dot < 0.95f)
        {
            float angle = acosf(dot);
            return ((*this)*sinf(angle*(1-t)) + q3*sinf(angle*t))/sinf(angle);
        } else {
            // if the angle is small, use linear interpolation                               
            return lerp(q3,t); 
        }      
    }
    
    const Vector3 getEulerAngles(){
        double sqw = w*w;
        double sqx = v.x*v.x;
        double sqy = v.y*v.y;
        double sqz = v.z*v.z;
        double unit = sqx + sqy + sqz + sqw;
        double test = v.x*v.y + v.z*w;
        Vector3 r;
        
        if (test > 0.499*unit) { // singularity at north pole
            r.z = 2 * atan2(v.x,w);
            r.x = M_PI/2;
            r.y = 0;
            return r;
        }
        if (test < -0.499*unit) { // singularity at south pole
            r.z = -2 * atan2(v.x,w);
            r.x = -M_PI/2;
            r.y = 0;
            return r;
        }
        r.z = atan2((double)(2*v.y*w-2*v.x*v.z ), (double)(sqx - sqy - sqz + sqw));
        r.x = asin(2*test/unit);
        r.y = atan2((double)(2*v.x*w-2*v.y*v.z) ,(double)( -sqx + sqy - sqz + sqw));
        
        return r;
    }
    
    Quaternion difference(const Quaternion &q2) const {
        return(Quaternion(q2*(*this).inverse()));
    }   
    
    

    //operators
    Quaternion &operator = (const Quaternion &q) {
        w = q.w;
        v = q.v;
        return *this;
    }

    const Quaternion operator + (const Quaternion &q) const {
        return Quaternion(w+q.w, v+q.v);
    }

    const Quaternion operator - (const Quaternion &q) const {
        return Quaternion(w - q.w, v - q.v);
    }

    const Quaternion operator * (const Quaternion &q) const {
        return Quaternion(w * q.w - v * q.v,
                          v.y * q.v.z - v.z * q.v.y + w * q.v.x + v.x * q.w,
                          v.z * q.v.x - v.x * q.v.z + w * q.v.y + v.y * q.w,
                          v.x * q.v.y - v.y * q.v.x + w * q.v.z + v.z * q.w);
    }
    
    const Quaternion operator / (const Quaternion &q) const {
        Quaternion p = q.inverse();
        return p;
    }
    
    const Quaternion operator - () const {
        return Quaternion(-w, -v);
    }
    
    //scaler operators
    const Quaternion operator * (float scaler) const {
        return Quaternion(w * scaler, v * scaler);
    }

    const Quaternion operator / (float scaler) const {
        return Quaternion(w / scaler, v / scaler);
    }    
    
    float w;
    Vector3 v;
    Vector3 r;      
};

#endif