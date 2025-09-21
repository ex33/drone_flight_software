#ifndef _MATHPK_H
#define _MATHPK_H




// Follows quaternion convention from Markley q = [qx; qy; qz; qw]
struct Quaternion {

    float x;
    float y;
    float z;
    float w;

    //Default constructor
    Quaternion() {
        x = 0.0;
        y = 0.0;
        z = 0.0;
        w = 1.0;
    };

    Quaternion(float qx, float qy, float qz, float qw) {
        x = qx;
        y = qy;
        z = qz;
        w = qw;
    }

    // Const to ensure q does not get changed
    Quaternion getProduct(const Quaternion& q) const {
        // Kronecker Product
        // Assumes quaternions are normalized
        return Quaternion(
            x*q.w - y*q.z + z*q.y + w*q.x,  //New x comp
            x*q.z + y*q.w - z*q.x + w*q.y,  //New y comp
            -x*q.y + y*q.x + z*q.w + w*q.z, //New z comp
            -x*q.x - y*q.y - z*q.z + w*q.w  //New w comp
        );
    }

    // Const to ensure old quaternion doesn't get overridden unless reassigned
    Quaternion getConjugate() const {
        return Quaternion(-x, -y, -z, w);
    }

    Quaternion getNorm() const {
        Quaternion quat (x,y,z,w);
        quat.norm();
        return quat;
    }

    float getMag() const {
        return sqrt(x*x + y*y + z*z + w*w);
    }

    void norm() {
        // Normalize Quaterion
        float mag = getMag();
        
        x /= mag;
        y /= mag;
        z /= mag;
        w /= mag;
    }
};





struct Vector3f {
    float x;
    float y;
    float z;

    //Default constructor
    Vector3f() {
        x=0;
        y=0;
        z=0;
    };

    Vector3f(float x_comp, float y_comp, float z_comp){
        x = x_comp;
        y = y_comp;
        z = z_comp;
    };

    // Functions
    float getMag() {
        return sqrt(x*x + y*y + z*z);
    };

    void norm() {
        float m = getMag();

        x /= m;
        y /=m;
        z /=m;
    };

    Vector3f getNorm() {
        Vector3f vec(x,y,z);
        vec.norm();
        return vec;
    };

    //Pass Reference of q to save memory rather than passing copy (q). 
    void rotate(const Quaternion& q) {
        // Rotate Vector given Quaternion according to Markley
        //  q x [v;0] x q*
        Quaternion vec_quat{x,y,z,0};
        
        //Do first Product
        vec_quat = q.getNorm().getProduct(vec_quat);

        // Do second Product
        vec_quat = vec_quat.getProduct(q.getNorm().getConjugate());
        
        x = vec_quat.x;
        y = vec_quat.y;
        z = vec_quat.z;
    };

    Vector3f getRotation(const Quaternion& q) {
        
        // Make a copy
        Vector3f vec {x, y, z};

        vec.rotate(q);
        
        return vec;
    };


};



#endif