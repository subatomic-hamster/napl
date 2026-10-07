#include <mujoco/mujoco.h>

#include <cmath>
#include <gtest/gtest.h>

TEST(smoke, pendulum_steps_finite) {
  char err[1000] = "";
  mjModel* m = mj_loadXML(NAPL_TEST_DATA_DIR "/pendulum.xml", nullptr, err, sizeof err);
  ASSERT_NE(m, nullptr) << err;
  mjData* d = mj_makeData(m);
  ASSERT_NE(d, nullptr);
  d->qpos[0] = 1.0;  // start off-equilibrium so it actually moves
  for (int i = 0; i < 1000; ++i) mj_step(m, d);
  for (int i = 0; i < m->nq; ++i) EXPECT_TRUE(std::isfinite(d->qpos[i])) << "qpos[" << i << "]";
  EXPECT_NE(d->qpos[0], 1.0);
  mj_deleteData(d);
  mj_deleteModel(m);
}
