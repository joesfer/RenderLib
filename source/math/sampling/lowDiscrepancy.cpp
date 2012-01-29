#include "math/sampling/lowDiscrepancy.h"

#define _USE_MATH_DEFINES // define M_PI
#include <math.h>

namespace RenderLib {
namespace Math {

	// Hammersley Points on 2D Plane with p1 = 2
	void planeHammersley(float *result, int n) {
		float p, u, v;
		int k, kk, pos;
		for (k=0, pos=0 ; k<n ; k++) {
			u = 0;
			for (p=0.5f, kk=k ; kk ; p*=0.5f, kk>>=1) {
				if (kk & 1) {// kk mod 2 == 1
					u += p;
				}
			}
			v = (k + 0.5f) / n;
			result[pos++] = u;
			result[pos++] = v;
		}
	}

	// Halton Points on 2D Plane with p1 = 2
	void planeHalton(float *result, int n, int p2) {
		float p, u, v, ip;
		int k, kk, pos, a;
		for (k=0, pos=0 ; k<n ; k++) {
			u = 0;
			for (p=0.5f, kk=k ; kk ; p*=0.5f, kk>>=1) {
				if (kk & 1) { // kk mod 2 == 1
					u += p;
				}
			}
			v = 0;
			ip = 1.0f/p2; // inverse of p2
			for (p=ip, kk=k ; kk ; p*=ip, kk/=p2) { // kk = (int)(kk/p2)
				if ((a = kk % p2)) {
					v += a * p;
				}
			}
			result[pos++] = u;
			result[pos++] = v;
		}
	}

	// Hammersley Points on Sphere with p1 = 2
	void sphereHammersley(float *result, int n) {
		float p, t, st, phi, phirad;
		int k, kk, pos;
		for (k=0, pos=0 ; k<n ; k++) {
			t = 0;
			for (p=0.5f, kk=k ; kk ; p*=0.5f, kk>>=1) {
				if (kk & 1) { // kk mod 2 == 1
					t += p;
				}
			}
			t = 2.0f * t - 1.0f; // map from [0,1] to [-1,1]
			phi = (k + 0.5f) / n; // a slight shift
			phirad = (float)(phi * 2.0 * M_PI); // map to [0, 2 pi)
			st = sqrtf(1.0f-t*t);
			result[pos++] = st * cosf(phirad);
			result[pos++] = st * sinf(phirad);
			result[pos++] = t;
		}
	}

	// Halton Points on Sphere with p1 = 2
	void sphereHalton(float *result, int n, int p2 ) {
		float p, t, st, phi, phirad, ip;
		int k, kk, pos, a;
		for (k=0, pos=0 ; k<n ; k++) {
			t = 0;
			for (p=0.5, kk=k ; kk ; p*=0.5, kk>>=1) {
				if (kk & 1) {// kk mod 2 == 1
					t += p;
				}
			}
			t = 2.0f * t - 1.0f; // map from [0,1] to [-1,1]
			st = sqrtf(1.0f-t*t);
			phi = 0;
			ip = 1.0f/p2; // inverse of p2
			for (p=ip, kk=k ; kk ; p*=ip, kk/=p2) {// kk = (int)(kk/p2)
				if ((a = kk % p2)) {
					phi += a * p;
				}
			}
			phirad = (float)(phi * 4.0 * M_PI); // map from [0,0.5] to [0, 2 pi)
			result[pos++] = st * cosf(phirad);
			result[pos++] = st * sinf(phirad);
			result[pos++] = t;
		}
	}

} // namespace Math
} // namespace RenderLib