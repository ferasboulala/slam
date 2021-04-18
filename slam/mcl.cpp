#include "mcl.h"

namespace slam
{

MCL::MCL(int n_particles, double alpha_fast, double alpha_slow)
    : m_omega_fast(1.0 / n_particles), m_omega_slow(1.0 / n_particles), m_alpha_fast(alpha_fast), m_alpha_slow(alpha_slow)
{
}

} // namespace slam
