
#include <boost/test/unit_test.hpp>

#define DYNOBENCH_BASE "../../dynobench/"

BOOST_AUTO_TEST_CASE(t_cli) {

  std::vector<std::string> _cmd = {
      "../main_dbrrt",
      "--env",
      DYNOBENCH_BASE + std::string("envs/unicycle1_v0/bugtrap_0.yaml"),
      "--models_base_path",
      DYNOBENCH_BASE + std::string("models/"),
      "--max_motions",
      "30",
      "--max_expands",
      "30000",
      "--seed",
      "1",
      "--do_optimization",
      "1",
      "--motionsFile",
      "../../data/motion_primitives/unicycle1_v0/"
      "unicycle1_v0__ispso__2023_04_03__14_56_57.bin.less.bin"
      // "motionsV2/good/unicycle1_v0/"
      // "unicycle1_v0__ispso__2023_04_03__14_56_57.bin"
  };

  std::string cmd = "";

  for (auto &c : _cmd) {
    cmd += c + " ";
  }

  int out0 = std::system("make ../main_dbrrt");
  BOOST_TEST(out0 == 0);

  std::cout << "Command is: " << cmd << std::endl;
  int out = std::system(cmd.c_str());
  BOOST_TEST(out == 0);
}
