


/////////////////////////////////////////////////////////////////////
/***************************GETTERS*********************************/
/////////////////////////////////////////////////////////////////////

void ForwardSearch::get_state(std::vector<Robot> robots,
                              volatile_map::VolatileMap volatile_map,
                              ros::Time time)
{
  robots_ = robots;
  volatile_map_ = volatile_map;
  time_ = time;

}


/////////////////////////////////////////////////////////////////////
/***************************CORE FUNTIONALITY***********************/
/////////////////////////////////////////////////////////////////////
int ForwardSearch::plan(std::vector<Robot> &robots,
          const volatile_map::VolatileMap &volatile_map,
          const ros::Time &time)
{

//T = Tree();
depth = 0;
while (depth < n-1)
{

  std::vector<vertex> temp;
  int i = 0;
  for(auto & leaf:tree[depth])
  {
    std::vection<int> a = get_actions(leaf);
    if (!a.empty())
    {
      std::vector<vertex> vert = expand(leaf,a);
      for (auto & v:vert)
      {
      leaf.children.push_back(i);
      //TODO: check this updates the leaf in the tree
      v.index = i;
      v.parent = leaf.index;
      v.depth = depth+1;
      temp.push_back(v);
      ++i;
      }
    }
  }

  if (!temp.empty())
  {
    tree.push_back(temp);
  }

  temp.clear();
  ++depth;
}

return get_policy();

}

bool ForwardSearch::reinit()
{

}

std::vector<vertex> ForwardSearch::expand(vertex v, int actions)
{
  std::vector<vertex> V;
  for (auto &a:actions)
  {
      vertex tempv = propagate(v,a);

      V.push_back(tempv);
  }
  return V;
}


vertex propagate(vertex v, int action)
{

  vertex u;

  //do things here...

  u.parent = v.index;


  return u;
}

std::vector<int> ForwardSearch::get_actions(vertex v, std::vector<int> a)
{

}

int ForwardSearch::get_policy()
{
  int max_ind = 0;
  doublt max_cost = -5e6;
  for (auto & v:tree[tree.size()-1])
  {
    if (max_cost < v.cost)
    {
      max_ind = v.index;
      max_cost = v.cost;
    }
  }

  return get_parent(max_ind, tree.size());
}

int ForwardSearch::get_parent(int ind_v,int depth)
{
  if (depth == 1)
  {
    return ind;
  } else
  {
    get_parent(tree[depth][ind_v].parent,depth-1);
  }
}



/////////////////////////////////////////////////////////////////////
/***************************CONSTRUCTORS****************************/
/////////////////////////////////////////////////////////////////////

ForwardSearch::ForwardSearch()
{

}

ForwardSearch::ForwardSearch(const CostFunction  cost_function,
                                  const PlanningParams planning_params): cost_function_(cost_function), robots_(robots), planning_params_(planning_params)
{

}


/////////////////////////////////////////////////////////////////////
/***************************UTILITIES*******************************/
/////////////////////////////////////////////////////////////////////
