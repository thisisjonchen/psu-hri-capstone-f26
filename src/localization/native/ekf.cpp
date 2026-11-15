#include "ekf.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

// Mean observations for initialization, rest detection and camera rotation only.
extern "C" int pals_ekf_imu_means(const double* q, const double* dv, double dt,
                                           double* force, double* rate) {
    double norm=0.;
    for (int i=0;i<4;++i) norm+=q[i]*q[i];
    if (!std::isfinite(norm) || std::abs(norm-1.)>.002 || q[3]<=0.
        || !std::isfinite(dt) || dt<=0. || dt>.2) return -1;
    for (int i=0;i<3;++i) if (!std::isfinite(dv[i])) return -1;
    norm=std::sqrt(norm);
    const double length=std::sqrt(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]);
    const double scale=length<1.e-10 ? 2./(norm*dt) : 2.*std::atan2(length,q[3])/(length*dt);
    const double hn=std::sqrt(2.*norm*(norm+q[3]));
    const double x=-q[0]/hn,y=-q[1]/hn,z=-q[2]/hn,w=(norm+q[3])/hn;
    const double tx=2*(y*dv[2]-z*dv[1]),ty=2*(z*dv[0]-x*dv[2]),tz=2*(x*dv[1]-y*dv[0]);
    force[0]=(dv[0]+w*tx+y*tz-z*ty)/dt;
    force[1]=(dv[1]+w*ty+z*tx-x*tz)/dt;
    force[2]=(dv[2]+w*tz+x*ty-y*tx)/dt;
    for (int i=0;i<3;++i) rate[i]=q[i]*scale;
    return 0;
}

namespace {
using Matrix = std::array<double, 225>;

Matrix multiply(const Matrix& a, const Matrix& b) {
    Matrix result = {};
    for (int i = 0; i < 15; ++i)
        for (int k = 0; k < 15; ++k)
            for (int j = 0; j < 15; ++j)
                result[15*i+j] += a[15*i+k]*b[15*k+j];
    return result;
}

Matrix transpose(const Matrix& a) {
    Matrix result;
    for (int i = 0; i < 15; ++i)
        for (int j = 0; j < 15; ++j)
            result[15*i+j] = a[15*j+i];
    return result;
}

void skew(const double* v, double* result) {
    const double values[] = {0., -v[2], v[1], v[2], 0., -v[0], -v[1], v[0], 0.};
    std::copy(values, values+9, result);
}

void rotation(const double* q, double* result) {
    const double norm = std::sqrt(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]);
    const double x=q[0]/norm, y=q[1]/norm, z=q[2]/norm, w=q[3]/norm;
    const double values[] = {
        1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w),
        2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w),
        2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)};
    std::copy(values, values+9, result);
}

void quaternion_product(const double* a, const double* b, double* result) {
    const double values[] = {
        a[3]*b[0]+a[0]*b[3]+a[1]*b[2]-a[2]*b[1],
        a[3]*b[1]-a[0]*b[2]+a[1]*b[3]+a[2]*b[0],
        a[3]*b[2]+a[0]*b[1]-a[1]*b[0]+a[2]*b[3],
        a[3]*b[3]-a[0]*b[0]-a[1]*b[1]-a[2]*b[2]};
    std::copy(values, values+4, result);
}

// State layout is position, velocity, then quaternion (x, y, z, w).
void nominal_step(double* state, const double* force, const double* rate,
                  const double* gravity, double step, double* mid_q, double* acceleration) {
    const double angle = std::sqrt(rate[0]*rate[0]+rate[1]*rate[1]+rate[2]*rate[2]);
    const double scale = angle < 1e-12 ? step/4 : std::sin(angle*step/4)/angle;
    const double half[] = {rate[0]*scale, rate[1]*scale, rate[2]*scale, std::cos(angle*step/4)};
    quaternion_product(state+6, half, mid_q);
    const double x=mid_q[0], y=mid_q[1], z=mid_q[2], w=mid_q[3];
    const double tx=2*(y*force[2]-z*force[1]);
    const double ty=2*(z*force[0]-x*force[2]);
    const double tz=2*(x*force[1]-y*force[0]);
    acceleration[0] = force[0]+w*tx+y*tz-z*ty+gravity[0];
    acceleration[1] = force[1]+w*ty+z*tx-x*tz+gravity[1];
    acceleration[2] = force[2]+w*tz+x*ty-y*tx+gravity[2];
    for (int i=0; i<3; ++i) {
        state[i] += state[i+3]*step+.5*acceleration[i]*step*step;
        state[i+3] += acceleration[i]*step;
    }
    quaternion_product(mid_q, half, state+6);
    double norm = 0.;
    for (int i=6; i<10; ++i) norm += state[i]*state[i];
    norm = std::sqrt(norm);
    for (int i=6; i<10; ++i) state[i] /= norm;
}

void propagate(double* covariance, const double* p, const double* v, const double* r,
               const double* gravity, const double* density, double step) {
    Matrix a = {}, noise, initial;
    double cross_p[9], cross_v[9], cross_g[9], gyro_map[27];
    skew(p, cross_p); skew(v, cross_v); skew(gravity, cross_g);
    for (int i=0; i<3; ++i) {
        a[15*i+i+3] = step;
        for (int j=0; j<3; ++j) {
            for (int k=0; k<3; ++k) {
                a[15*i+j+12] -= cross_p[3*i+k]*r[3*k+j]*step;
                a[15*(i+3)+j+12] -= cross_v[3*i+k]*r[3*k+j]*step;
            }
            a[15*(i+3)+j+6] = cross_g[3*i+j]*step;
            a[15*(i+3)+j+9] = -r[3*i+j]*step;
            a[15*(i+6)+j+12] = -r[3*i+j]*step;
            gyro_map[3*i+j] = cross_p[3*i+j];
            gyro_map[3*(i+3)+j] = cross_v[3*i+j];
            gyro_map[3*(i+6)+j] = i == j ? 1. : 0.;
        }
    }
    const Matrix a2=multiply(a, a), a3=multiply(a2, a);
    Matrix transition, half;
    for (int i=0; i<225; ++i) {
        const double identity = i/15 == i%15 ? 1. : 0.;
        transition[i] = identity+a[i]+a2[i]/2+a3[i]/6;
        half[i] = identity+a[i]/2+a2[i]/8+a3[i]/48;
        noise[i] = density[i];
    }
    for (int i=0; i<9; ++i)
        for (int j=0; j<9; ++j) {
            double value = 0.;
            for (int k=0; k<3; ++k) value += gyro_map[3*i+k]*gyro_map[3*j+k];
            noise[15*i+j] = density[15*6+6]*value;
        }
    for (int i=3; i<6; ++i)
        for (int j=3; j<6; ++j) noise[15*i+j] += density[15*i+j];
    for (int i=0; i<225; ++i) {
        noise[i] *= step/6;
        initial[i] = covariance[i]+noise[i];
    }
    const Matrix first=multiply(multiply(transition, initial), transpose(transition));
    const Matrix middle=multiply(multiply(half, noise), transpose(half));
    for (int i=0; i<225; ++i) covariance[i] = first[i]+4*middle[i]+noise[i];
}

void symmetrize(double* covariance) {
    for (int i=0; i<15; ++i)
        for (int j=i; j<15; ++j) {
            const double value = .5*(covariance[15*i+j]+covariance[15*j+i]);
            covariance[15*i+j] = covariance[15*j+i] = value;
        }
}

bool finite(const double* values, std::size_t count) {
    for (std::size_t i=0; i<count; ++i)
        if (!std::isfinite(values[i])) return false;
    return true;
}
} // namespace

// The Python caller owns these buffers and commits them only on success.
extern "C" int pals_ekf_predict(double* state, double* covariance, const double* force,
                                const double* rate, const double* gravity,
                                const double* density, double duration) {
    if (!std::isfinite(duration) || duration <= 0. || duration > 1.) return 1;
    const int steps = std::max(1, static_cast<int>(std::ceil(duration/.01-1e-10)));
    const double step = duration/steps;
    for (int n=0; n<steps; ++n) {
        double mid_p[3], mid_v[3], mid_q[4], acceleration[3], r[9];
        std::copy(state, state+3, mid_p);
        std::copy(state+3, state+6, mid_v);
        nominal_step(state, force, rate, gravity, step, mid_q, acceleration);
        for (int i=0; i<3; ++i) {
            mid_p[i] += mid_v[i]*step/2+acceleration[i]*step*step/8;
            mid_v[i] += acceleration[i]*step/2;
        }
        rotation(mid_q, r);
        propagate(covariance, mid_p, mid_v, r, gravity, density, step);
    }
    if (!finite(state, 10) || !finite(covariance, 225)) return 1;
    symmetrize(covariance);
    return 0;
}

namespace {
void rotation_vector(const double* q, double* angle) {
    const double n=std::sqrt(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]);
    const double scale=n<1.e-12 ? 2. : 2.*std::atan2(n,q[3])/n;
    for (int i=0;i<3;++i) angle[i]=q[i]*scale;
}
void from_rotation_vector(const double* angle, double fraction, double* q) {
    const double n=std::sqrt(angle[0]*angle[0]+angle[1]*angle[1]+angle[2]*angle[2]);
    const double scale=n<1.e-12 ? fraction*.5 : std::sin(n*fraction*.5)/n;
    for (int i=0;i<3;++i) q[i]=angle[i]*scale;
    q[3]=std::cos(n*fraction*.5);
}
void rotate_vector(const double* q, const double* v, double* out) {
    double r[9]; rotation(q,r);
    for (int i=0;i<3;++i) out[i]=r[3*i]*v[0]+r[3*i+1]*v[1]+r[3*i+2]*v[2];
}

struct IntegratedMotion {
    double start, end, duration, angle[3], velocity[3];
    bool set(const double* packet, const double* ba, const double* bg) {
        if (!finite(packet,9) || !finite(ba,3) || !finite(bg,3)) return false;
        start=packet[0]; end=packet[1]; duration=end-start;
        const double* q=packet+2;
        double norm=0.; for (int i=0;i<4;++i) norm+=q[i]*q[i];
        if (!(duration>0. && duration<=.200000001) || std::abs(norm-1.)>.002 || q[3]<=0.) return false;
        double normalized[4], raw_angle[3], half[4], mid_velocity[3];
        for (int i=0;i<4;++i) normalized[i]=q[i]/std::sqrt(norm);
        rotation_vector(normalized,raw_angle);
        from_rotation_vector(raw_angle,-.5,half);
        rotate_vector(half,packet+6,mid_velocity);
        // Biases are body-frame rates. Apply their duration-scaled correction at the midpoint.
        for (int i=0;i<3;++i) {
            angle[i]=raw_angle[i]-bg[i]*duration;
            mid_velocity[i]-=ba[i]*duration;
        }
        from_rotation_vector(angle,.5,half);
        rotate_vector(half,mid_velocity,velocity);
        return finite(angle,3) && finite(velocity,3);
    }
    void advance(double* state, const double* gravity, double first, double last,
                 double* mid_q, double* acceleration) const {
        const double dt=last-first, fraction=dt/duration;
        double inverse_start[4], body_velocity[3], local_velocity[3], dq[4];
        // Interpolate velocity linearly in interval-start axes. Slices compose to the full delta.
        from_rotation_vector(angle,-(first-start)/duration,inverse_start);
        rotate_vector(inverse_start,velocity,body_velocity);
        rotate_vector(state+6,body_velocity,local_velocity);
        from_rotation_vector(angle,.5*fraction,dq);
        quaternion_product(state+6,dq,mid_q);
        from_rotation_vector(angle,fraction,dq);
        quaternion_product(state+6,dq,state+6);
        double norm=0.; for (int i=6;i<10;++i) norm+=state[i]*state[i];
        for (int i=6;i<10;++i) state[i]/=std::sqrt(norm);
        for (int i=0;i<3;++i) {
            const double dv=local_velocity[i]*fraction+gravity[i]*dt;
            acceleration[i]=dv/dt;
            state[i]+=state[i+3]*dt+.5*dv*dt;
            state[i+3]+=dv;
        }
    }
};
}

extern "C" int pals_ekf_predict_integrated(double* state, double* covariance, const double* packet,
                                            const double* gravity, const double* density,
                                            double start, double end) {
    IntegratedMotion motion;
    if (!motion.set(packet,state+10,state+13) || !std::isfinite(start) || !std::isfinite(end)
        || !(start<end) || start<motion.start-1.e-9 || end>motion.end+1.e-9) return 1;
    const int steps=std::max(1,static_cast<int>(std::ceil((end-start)/.01-1.e-10)));
    const double step=(end-start)/steps;
    for (int n=0;n<steps;++n) {
        double mid_p[3],mid_v[3],mid_q[4],acceleration[3],r[9];
        std::copy(state,state+3,mid_p); std::copy(state+3,state+6,mid_v);
        motion.advance(state,gravity,start+n*step,start+(n+1)*step,mid_q,acceleration);
        for (int i=0;i<3;++i) {
            mid_p[i]+=mid_v[i]*step/2+acceleration[i]*step*step/8;
            mid_v[i]+=acceleration[i]*step/2;
        }
        rotation(mid_q,r);
        propagate(covariance,mid_p,mid_v,r,gravity,density,step);
    }
    if (!finite(state,16) || !finite(covariance,225)) return 2;
    symmetrize(covariance);
    return 0;
}

extern "C" int pals_ekf_project_integrated(double* state, double* covariance, const double* intervals,
                                            std::size_t count, const double* gravity,
                                            const double* density, double start) {
    if (!count) return 0;
    const double last=intervals[9*(count-1)+1],duration=last-start;
    // Must exceed the node's delayed-fusion guard (fusion delay + 0.35 s).
    if (!std::isfinite(start) || !std::isfinite(duration) || duration<=0. || duration>1.) return 1;
    const double midpoint=start+duration/2;
    double stamp=start,mid_state[10],r[9]; bool have_midpoint=false;
    for (std::size_t i=0;i<count;++i) {
        IntegratedMotion motion;
        if (!motion.set(intervals+9*i,state+10,state+13) || !(stamp<motion.end && motion.end<=last)
            || (i ? std::abs(stamp-motion.start)>1.e-9 : stamp<motion.start-1.e-9)) return 2;
        const bool split=stamp<midpoint && midpoint<motion.end;
        const double stops[]={split ? midpoint : motion.end,motion.end};
        for (int part=0;part<(split ? 2 : 1);++part) {
            double mid_q[4],acceleration[3];
            motion.advance(state,gravity,stamp,stops[part],mid_q,acceleration);
            stamp=stops[part];
            if (stamp==midpoint) {
                std::copy(state,state+10,mid_state); have_midpoint=true;
            }
        }
    }
    if (!have_midpoint) return 2;
    rotation(mid_state+6,r);
    propagate(covariance,mid_state,mid_state+3,r,gravity,density,duration);
    if (!finite(state,16) || !finite(covariance,225)) return 3;
    symmetrize(covariance);
    return 0;
}

extern "C" int pals_ekf_vertical_output(double* output, double height, double velocity,
                                          double delta_velocity, double dt) {
    if (!finite(output,2) || !std::isfinite(height) || !std::isfinite(velocity)
        || !std::isfinite(delta_velocity) || !std::isfinite(dt) || dt<=0. || dt>.5) return 1;
    const double decay=std::exp(-dt/.1);
    // Follow IMU motion immediately; spread EKF corrections over successive samples.
    // Position corrections must not be mistaken for physical climb rate.
    const double predicted_height=output[0]+(output[1]+.5*delta_velocity)*dt;
    const double predicted_velocity=output[1]+delta_velocity;
    const double position=height+decay*(predicted_height-height);
    const double rate=velocity+decay*(predicted_velocity-velocity);
    if (!std::isfinite(position) || !std::isfinite(rate)) return 2;
    output[0]=position; output[1]=rate;
    return 0;
}

namespace {
// Solve all right-hand sides together, using partial pivoting.
bool solve(double* a, double* rhs, std::size_t n, std::size_t columns) {
    for (std::size_t k=0; k<n; ++k) {
        std::size_t pivot=k;
        for (std::size_t i=k+1; i<n; ++i)
            if (std::abs(a[i*n+k]) > std::abs(a[pivot*n+k])) pivot=i;
        if (a[pivot*n+k] == 0. || !std::isfinite(a[pivot*n+k])) return false;
        for (std::size_t j=0; j<n; ++j) std::swap(a[k*n+j], a[pivot*n+j]);
        for (std::size_t j=0; j<columns; ++j) std::swap(rhs[k*columns+j], rhs[pivot*columns+j]);
        for (std::size_t i=k+1; i<n; ++i) {
            const double factor=a[i*n+k]/a[k*n+k];
            for (std::size_t j=k+1; j<n; ++j) a[i*n+j] -= factor*a[k*n+j];
            for (std::size_t j=0; j<columns; ++j) rhs[i*columns+j] -= factor*rhs[k*columns+j];
        }
    }
    for (std::size_t row=n; row>0; --row) {
        const std::size_t i=row-1;
        for (std::size_t j=0; j<columns; ++j) {
            for (std::size_t k=i+1; k<n; ++k) rhs[i*columns+j] -= a[i*n+k]*rhs[k*columns+j];
            rhs[i*columns+j] /= a[i*n+i];
        }
    }
    return true;
}

struct CameraGeometry {
    double r[9], height;
    CameraGeometry(const double* p, const double* q, const double* offset, double floor) {
        rotation(q, r);
        height=p[2]-floor;
        for (int j=0; j<3; ++j) height += r[6+j]*offset[j];
    }
    bool ray(const double* input, const double* mount, double min_height, double max_reach,
             double* body, double* direction) const {
        for (int i=0; i<3; ++i) {
            body[i]=0.;
            for (int j=0; j<3; ++j) body[i] += mount[3*i+j]*input[j];
        }
        for (int i=0; i<3; ++i) {
            direction[i]=0.;
            for (int j=0; j<3; ++j) direction[i] += r[3*i+j]*body[j];
        }
        const double depth=height/(direction[2] < -1e-3 ? -direction[2] : 1.);
        const double x=depth*direction[0], y=depth*direction[1];
        return direction[2] < -1e-3 && height >= min_height && std::sqrt(x*x+y*y) <= max_reach;
    }
};

bool valid_quaternion(const double* q) {
    double norm=0.;
    for (int i=0; i<4; ++i) norm += q[i]*q[i];
    return std::isfinite(norm) && norm >= 1e-12;
}
} // namespace

// Correction state also carries accelerometer and gyro biases after the quaternion.
// Status 2 fails before consuming the measurement; 3 fails after its innovation gate.
extern "C" int pals_ekf_correct(double* state, double* covariance, const double* innovation,
                                const double* observation, const double* noise, std::size_t size,
                                double gate, int xy_only, double* total, double* nis) {
    if (!size) return 2;
    double r[9], cross_p[9], cross_v[9];
    rotation(state+6, r); skew(state, cross_p); skew(state+3, cross_v);
    Matrix transform = {}, prior;
    for (int i=0; i<15; ++i) transform[15*i+i]=1.;
    for (int i=0; i<3; ++i)
        for (int j=0; j<3; ++j) {
            transform[15*i+j+6]=-cross_p[3*i+j];
            transform[15*(i+3)+j+6]=-cross_v[3*i+j];
            transform[15*(i+6)+j+6]=r[3*j+i];
        }
    std::copy(covariance, covariance+225, prior.begin());
    std::vector<double> h(size*15,0.), hp(size*15,0.), rhs(size*16), a(size*size);
    for (std::size_t i=0; i<size; ++i)
        for (int k=0; k<15; ++k)
            for (int j=0; j<15; ++j) h[15*i+j] += observation[15*i+k]*transform[15*k+j];
    for (std::size_t i=0; i<size; ++i)
        for (int k=0; k<15; ++k)
            for (int j=0; j<15; ++j) hp[15*i+j] += h[15*i+k]*prior[15*k+j];
    for (std::size_t i=0; i<size; ++i) {
        rhs[16*i]=innovation[i];
        for (int j=0; j<15; ++j) rhs[16*i+j+1]=hp[15*i+j];
        for (std::size_t j=0; j<size; ++j) {
            double value=noise[size*i+j];
            for (int k=0; k<15; ++k) value += hp[15*i+k]*h[15*j+k];
            a[size*i+j]=total[size*i+j]=value;
        }
    }
    if (!solve(a.data(), rhs.data(), size, 16)) return 2;
    *nis=0.;
    for (std::size_t i=0; i<size; ++i) *nis += innovation[i]*rhs[16*i];
    if (!std::isfinite(*nis)) return 2;
    if (*nis > gate) return 1;
    if (xy_only) {
        // Restrict in public coordinates; internal position error includes p x attitude.
        for (std::size_t k=0; k<size; ++k) {
            double* gain=&rhs[16*k+1];
            const double x=gain[0]-cross_p[0]*gain[6]-cross_p[1]*gain[7]-cross_p[2]*gain[8];
            const double y=gain[1]-cross_p[3]*gain[6]-cross_p[4]*gain[7]-cross_p[5]*gain[8];
            std::fill(gain,gain+15,0.); gain[0]=x; gain[1]=y;
        }
    }
    double delta[15] = {};
    Matrix remainder = {};
    for (int i=0; i<15; ++i) {
        remainder[15*i+i]=1.;
        for (std::size_t k=0; k<size; ++k) {
            const double gain=rhs[16*k+i+1];
            delta[i] += gain*innovation[k];
            for (int j=0; j<15; ++j) remainder[15*i+j] -= gain*h[15*k+j];
        }
    }
    const Matrix updated=multiply(multiply(remainder, prior), transpose(remainder));
    for (int i=0; i<15; ++i)
        for (int j=0; j<15; ++j) {
            double value=updated[15*i+j];
            for (std::size_t k=0; k<size; ++k)
                for (std::size_t l=0; l<size; ++l)
                    value += rhs[16*k+i+1]*noise[size*k+l]*rhs[16*l+j+1];
            covariance[15*i+j]=value;
        }
    if (!finite(delta,15) || !finite(covariance,225)) return 3;
    if (xy_only) {
        state[0]+=delta[0];
        state[1]+=delta[1];
        symmetrize(covariance);
        return finite(state,2) ? 0 : 3;
    }
    const double angle=std::sqrt(delta[6]*delta[6]+delta[7]*delta[7]+delta[8]*delta[8]);
    const double scale=angle < 1e-6 ? .5-angle*angle/48 : std::sin(angle/2)/angle;
    const double correction[]={scale*delta[6],scale*delta[7],scale*delta[8],std::cos(angle/2)};
    const double ja=angle < 1e-4 ? .5-angle*angle/24 : (1-std::cos(angle))/(angle*angle);
    const double jb=angle < 1e-4 ? 1./6-angle*angle/120 : (angle-std::sin(angle))/(angle*angle*angle);
    double cross[9], translation[9], updated_state[6]={};
    skew(delta+6,cross); rotation(correction,r);
    for (int i=0; i<3; ++i)
        for (int j=0; j<3; ++j) {
            translation[3*i+j]=(i==j ? 1. : 0.)+ja*cross[3*i+j];
            for (int k=0; k<3; ++k) translation[3*i+j] += jb*cross[3*i+k]*cross[3*k+j];
        }
    for (int block=0; block<2; ++block)
        for (int i=0; i<3; ++i)
            for (int j=0; j<3; ++j)
                updated_state[3*block+i] += r[3*i+j]*state[3*block+j]+translation[3*i+j]*delta[3*block+j];
    std::copy(updated_state,updated_state+6,state);
    quaternion_product(correction,state+6,state+6);
    double norm=0.;
    for (int i=6; i<10; ++i) norm += state[i]*state[i];
    norm=std::sqrt(norm);
    for (int i=6; i<10; ++i) state[i] /= norm;
    for (int i=0; i<6; ++i) state[10+i] += delta[9+i];
    symmetrize(covariance);
    return 0;
}

extern "C" std::ptrdiff_t pals_camera_indices(const double* p, const double* q, const double* mount,
    const double* offset, const double* rays, std::size_t count, double floor, double min_height,
    double max_reach, std::int64_t* indices) {
    if (!valid_quaternion(q)) return -1;
    const CameraGeometry geometry(p,q,offset,floor);
    std::ptrdiff_t used=0;
    for (std::size_t i=0; i<count; ++i) {
        double body[3], direction[3];
        if (geometry.ray(rays+3*i,mount,min_height,max_reach,body,direction)) indices[used++]=i;
    }
    return used;
}

extern "C" std::ptrdiff_t pals_camera_linearize(const double* p, const double* v, const double* q,
    const double* bias, const double* rate, const double* mount, const double* offset,
    const double* rays, std::size_t count, double floor, double min_height, double max_reach,
    std::int64_t* indices, double* prediction, double* observation) {
    if (!valid_quaternion(q)) return -1;
    const CameraGeometry geometry(p,q,offset,floor);
    double body_v[3]={}, omega[3], sensor_v[3], offset_skew[9], velocity_skew[9], height_jacobian[3]={};
    skew(offset,offset_skew);
    for (int i=0; i<3; ++i) {
        omega[i]=rate[i]-bias[i];
        for (int j=0; j<3; ++j) {
            body_v[i] += geometry.r[3*j+i]*v[j];
            height_jacobian[i] -= geometry.r[6+j]*offset_skew[3*j+i];
        }
    }
    skew(body_v,velocity_skew);
    for (int i=0; i<3; ++i) {
        sensor_v[i]=body_v[i];
        for (int j=0; j<3; ++j) sensor_v[i] -= offset_skew[3*i+j]*omega[j];
    }
    std::ptrdiff_t used=0;
    for (std::size_t n=0; n<count; ++n) {
        const double* ray=rays+3*n;
        double body[3], direction[3];
        if (!geometry.ray(ray,mount,min_height,max_reach,body,direction)) continue;
        indices[used]=n;
        const double inverse_depth=-direction[2]/geometry.height;
        const double projection[]={1.,0.,-ray[0],0.,1.,-ray[1]};
        double camera[6]={}, angular[6]={}, ray_skew[9], body_skew[9], depth_jacobian[3]={};
        skew(ray,ray_skew); skew(body,body_skew);
        for (int i=0; i<2; ++i)
            for (int j=0; j<3; ++j)
                for (int k=0; k<3; ++k) {
                    camera[3*i+j] += projection[3*i+k]*mount[3*j+k];
                    for (int l=0; l<3; ++l)
                        angular[3*i+j] -= projection[3*i+k]*ray_skew[3*k+l]*mount[3*j+l];
                }
        for (int j=0; j<3; ++j) {
            for (int k=0; k<3; ++k) depth_jacobian[j] += geometry.r[6+k]*body_skew[3*k+j];
            depth_jacobian[j]=(depth_jacobian[j]-inverse_depth*height_jacobian[j])/geometry.height;
        }
        for (int i=0; i<2; ++i) {
            double translated=0., angular_rate=0.;
            for (int j=0; j<3; ++j) {
                translated += camera[3*i+j]*sensor_v[j];
                angular_rate += angular[3*i+j]*omega[j];
            }
            prediction[2*used+i]=inverse_depth*translated+angular_rate;
            double* h=observation+30*used+15*i;
            std::fill(h,h+15,0.);
            h[2]=-inverse_depth*translated/geometry.height;
            for (int j=0; j<3; ++j) {
                h[6+j]=translated*depth_jacobian[j];
                h[12+j]=-angular[3*i+j];
                for (int k=0; k<3; ++k) {
                    h[3+j] += inverse_depth*camera[3*i+k]*geometry.r[3*j+k];
                    h[6+j] += inverse_depth*camera[3*i+k]*velocity_skew[3*k+j];
                    h[12+j] += inverse_depth*camera[3*i+k]*offset_skew[3*k+j];
                }
            }
        }
        ++used;
    }
    return used;
}
