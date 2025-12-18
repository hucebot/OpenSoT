#pragma once

#include <string>
#include <vector>
#include <list>
#include <unordered_map>

namespace OpenSoT::oc
{

    class Scheduler
    {
        typedef std::string str;

    private:
        std::unordered_map<str, std::vector<str>> _contacts;

        struct Phase
        {
            double duration;      // time duration in ms
            double starting_time; //
            double end_time;
            std::vector<str> contact_frames; // frames in contact
        };

        struct Sequence
        {
            std::list<Phase> phases;
            double duration;
        };

        std::unordered_map<str, Sequence> _sequences; // phase sequence

    public:
        Scheduler();

        void addContact(std::string contact_name, std::vector<std::string> contact_frame_names);

        void addPhase(std::vector<std::string> contacts_list, double duration, std::string sequence_name = "_");

        int getNodesNumber();

        std::list<std::vector<str>> getSequence(double sampling_rate, str sequence_name = "_", int nodes_number = -1, double current_time = 0.);
    };

} // namespace OpenSoT::oc