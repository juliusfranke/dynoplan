#include <boost/test/unit_test.hpp>

// #define DYNOBENCH_BASE "../../dynobench/dynobench/"
#define DYNOBENCH_BASE "../../dynobench/"

BOOST_AUTO_TEST_CASE(t_cli) {

  std::vector<std::string> _cmd = {
      "../main_idbastar",
      "--env_file",
      DYNOBENCH_BASE "envs/unicycle1_v0/bugtrap_0.yaml",
      "--motionsFile",
      "../../data/motion_primitives/unicycle1_v0/"
      "unicycle1_v0__ispso__2023_04_03__14_56_57.bin.less.bin",
      "--solver_id",
      "1",
      "--cost_delta_factor",
      "1",
      "--num_primitives_0",
      "30",
      "--models_base_path",
      DYNOBENCH_BASE "models/"};

  //     "/home/quim/stg/wolfgang/kinodynamic-motion-planning-benchmark/cloud/"
  //     "motionsV2/good/unicycle1_v0/"
  //     "unicycle1_v0__ispso__2023_04_03__14_56_57.bin",
  //     "--use_nigh_nn",
  //     "1",
  //     "--models_base_path",
  //     DYNOBENCH_BASE + std::string("models/")};

  std::string cmd = "";

  for (auto &c : _cmd) {
    cmd += c + " ";
  }

  std::cout << "Command is: " << cmd << std::endl;
  int out = std::system(cmd.c_str());
  BOOST_TEST(out == 0);
}
