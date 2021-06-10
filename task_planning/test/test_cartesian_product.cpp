
#include <iostream>
#include <vector>
#include <numeric>
#include <string>

std::vector<std::vector<int>> cartesian_product(const std::vector<std::vector<int>> &v)
{
  std::vector<std::vector<int>> s = {{}};
  for (const auto &u : v)
  {
    std::vector<std::vector<int>> r;
    for (const auto &x : s)
    {
      for (const auto &y : u)
      {
        r.push_back(x);
        r.back().push_back(y);
      }
    }
    s = std::move(r);
  }
  return s;
}

void print_vector(const std::vector<int> &v, std::string s)
{
  std::cout << s << ":";
  for (int i = 0; i < v.size(); i++)
  {
    std::cout << v[i] << " ";
  }
  std::cout << std::endl;
};

void print_vector(const std::vector<double> &v, std::string s)
{
  std::cout << s << ":";
  for (int i = 0; i < v.size(); i++)
  {
    std::cout << v[i] << " ";
  }
  std::cout << std::endl;
};

int main(int argc, char **argv)
{
  std::vector<double> Ad = {-1.1, 2.7, 3};
  std::vector<double> Bd = {500};
  std::vector<double> Cd = {0.097, 0.002};
  std::cout << "INPUT VECTOR INDICES" << std::endl;
  print_vector(Ad, "Ad");
  print_vector(Bd, "Bd");
  print_vector(Cd, "Cd");
  std::vector<std::vector<double>> Sd;
  Sd.push_back(Ad);
  Sd.push_back(Bd);
  Sd.push_back(Cd);

  std::vector<int> A(3);
  std::vector<int> B(1);
  std::vector<int> C(2);
  std::iota(A.begin(), A.end(), 0);
  std::iota(B.begin(), B.end(), 0);
  std::iota(C.begin(), C.end(), 0);
  std::cout << "INPUT VECTOR INDICES" << std::endl;
  print_vector(A, "A");
  print_vector(B, "B");
  print_vector(C, "C");
  std::vector<std::vector<int>> S;
  S.push_back(A);
  S.push_back(B);
  S.push_back(C);

  std::vector<std::vector<int>> prod;
  prod = cartesian_product(S);

  std::cout << "CARTESIAN PRODUCT" << std::endl;
  for (int i = 0; i < prod.size(); i++)
  {
    std::cout << "prod[" << i << "] = ";
    for (int j = 0; j < prod[i].size(); j++)
    {
      std::cout << prod[i][j] << " ";
    }
    std::cout << std::endl;
  }

  std::cout << "CARTESIAN PRODUCT MAPPED TO INPUTS" << std::endl;
  std::vector<std::vector<double>> prodd;
  for (int i = 0; i < prod.size(); i++)
  {
    std::vector<double> d;
    for (int j = 0; j < prod[i].size(); j++)
    {
      d.push_back(Sd[j][prod[i][j]]);
    }
    prodd.push_back(d);
  }
  for (int i = 0; i < prodd.size(); i++)
  {
    std::cout << "prodd[" << i << "] = ";
    for (int j = 0; j < prodd[i].size(); j++)
    {
      std::cout << prodd[i][j] << " ";
    }
    std::cout << std::endl;
  }

  return 0;
}
