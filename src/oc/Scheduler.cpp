#include "OpenSoT/oc/Scheduler.h"
#include <cmath>

using namespace OpenSoT::oc;

Scheduler::Scheduler()
{
}

void Scheduler::addContact(str contact_name, std::vector<str> contact_frame_names)
{
    _contacts[contact_name] = contact_frame_names;
}

void Scheduler::addPhase(std::vector<str> contacts_list, double duration, str sequence_name)
{
    Phase phase;
    phase.duration = duration;

    for (int i = 0; i < contacts_list.size(); i++)
        phase.contact_frames.insert(phase.contact_frames.end(),
                                    _contacts[contacts_list.at(i)].begin(),
                                    _contacts[contacts_list.at(i)].end());
    phase.starting_time = _sequences[sequence_name].duration;
    phase.end_time = phase.starting_time + duration;

    _sequences[sequence_name].phases.emplace_back(phase);
    _sequences[sequence_name].duration += phase.duration;
}

std::list<std::vector<std::string>> Scheduler::getSequence(double sampling_rate, str sequence_name, int nodes_number, double current_time)
{
    double dt = sampling_rate;
    int n_nodes = 0;
    std::list<std::vector<str>> contact_frames_sequence;

    Sequence _seq = _sequences[sequence_name];
    
    double t = current_time;
    double total_duration = _seq.duration;

    while (nodes_number == -1 || n_nodes < nodes_number)
    {
        // Wrap time within sequence duration
        double wrapped_time = std::fmod(t, total_duration);
        
        // Find which phase this time falls into
        for (auto& phase : _seq.phases)
        {
            if (wrapped_time < phase.end_time)
            {
                contact_frames_sequence.emplace_back(phase.contact_frames);
                break;
            }
        }
        
        n_nodes++;
        t += dt;
        
        if (nodes_number == -1 && wrapped_time + dt >= total_duration)
            break; // Completed one full sequence
    }

    return contact_frames_sequence;
}