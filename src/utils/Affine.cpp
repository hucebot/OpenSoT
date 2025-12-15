#include <OpenSoT/utils/Affine.h>


int OpenSoT::OptvarHelper::getSize() const
{
    return _size;
}

OpenSoT::OptvarHelper::OptvarHelper(std::vector< std::pair< std::string, int > > name_size_pairs)
{
    _size = 0;
    
    for(auto pair : name_size_pairs){

        if( _vars_map.count(pair.first) ){
            throw std::invalid_argument("Duplicate variable names are not allowed");
        }
            
        
        VarInfo vinfo;
        vinfo.size = pair.second;
        vinfo.start_idx = _size;
        vinfo.name = pair.first;
        
        _size += vinfo.size;
        
        _vars.push_back(vinfo);
        _vars_map[pair.first] = vinfo;
        
    }
}

OpenSoT::VariableXd OpenSoT::OptvarHelper::getVariable(std::string name) const
{
    auto it = _vars_map.find(name);
    
    if( it == _vars_map.end() ){
        throw std::invalid_argument("Variable does not exist");
    }

    return OpenSoT::VariableXd(it->second.start_idx, _size, it->second.size);
}

std::vector< OpenSoT::VariableXd > OpenSoT::OptvarHelper::getAllVariables() const
{
    std::vector<OpenSoT::VariableXd> all_vars;
    for(auto v : _vars){
        all_vars.push_back( getVariable(v.name) );
    }
    
    return all_vars;
}



























































