#pragma once
#include <string>
#include <map>

namespace askinect {

  class RecognizeResults {
    private:
      std::map<std::string, int> counts;

    public:
      RecognizeResults() {}

      ~RecognizeResults() {}

      void addResults(std::string id, int count) {
        counts[id] = count;
      }

      int getResults(std::string id) {
        return counts[id];
      }
  };

}