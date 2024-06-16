#ifndef A4136B41_6469_4E7F_982B_AF316A486310
#define A4136B41_6469_4E7F_982B_AF316A486310


#include <string>
#include <vector>

class IJsonLoadable
{
public:
	virtual void LoadConfig(const std::string& InJsonFilePath, const std::vector<std::string>& InConfigKeyList) = 0;
};

#endif /* A4136B41_6469_4E7F_982B_AF316A486310 */
