import rospy, smach, smach_ros, actionlib
from std_msgs.msg import Empty

class AbstractService():

    def __init__(self, name, spec):
        self._server = rospy.Service(name, spec, self.execute_cb)


class AbstractActionServer():

    def __init__(self, name, spec, feedback, result):
        self._feedback = feedback
        self._result = result
        self._as = actionlib.SimpleActionServer(name, spec, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()


class MonitoredMode(smach.Concurrence):

    def __init__(self, outcomes, default_outcome, monitored_topics,
                input_keys=[], output_keys=[], outcome_map={},
                outcome_cb=None, child_term_cb=None):

        self._monitored_topics = monitored_topics
        self._mode_outcome_map = outcome_map
        self._user_outcome_cb = outcome_cb
        self._user_child_term_cb = child_term_cb

        for topic in self._monitored_topics:
            self._mode_outcome_map[topic + '_preempt'] = {topic.upper() + '_MONITOR': 'valid'}
            outcomes.append(topic + '_preempt')

        smach.Concurrence.__init__(self, outcomes, default_outcome, input_keys, output_keys, {}, self._mode_outcome_cb, self._mode_child_term_cb)

        with self:
            for topic in self._monitored_topics:
                smach.Concurrence.add(topic.upper() + '_MONITOR', smach_ros.MonitorState(topic, Empty, lambda ud, msg: True, max_checks=1))


    def _mode_outcome_cb(self, outcome_map):
        # Checking for preemptions first is more important, so we'll check all the preemption topics before regular ones
        for preempt_outcome in [po for po in self._mode_outcome_map if po.endswith('_preempt')]:
            for state in [s for s in outcome_map if s.endswith('_MONITOR') and s in self._mode_outcome_map[preempt_outcome]]:
                # This says that if the preempt monitor state's expected outcome (valid) equals the actual outcome
                # for that state, then we've found a preempt outcome and should return it.
                # It's alright that this returns the first one it finds. If multiple preempt monitors returned valid
                # and we transition to one with a lower priority, once we reach that error handler it will in turn
                # be preempted by the error handler with a higher priority. In other words, it will eventually reach the
                # error handler with the appropriate priority.
                if self._mode_outcome_map[preempt_outcome][state] == outcome_map[state]:
                    return preempt_outcome

        # If we're here then the concurrence completed without any preempts occuring. We can finish acording to the user's
        # outcome callback if it exists. Otherwise we'll just use the original outcome callback map if one was provided.
        # Otherwise we'll just return the default outcome
        if self._user_outcome_cb:
            return self._user_outcome_cb(outcome_map)
        elif len([bo for bo in self._mode_outcome_map if not bo.endswith('_preempt')]) > 0:
            for basic_outcome in [bo for bo in self._mode_outcome_map if not bo.endswith('_preempt')]:
                if self._mode_outcome_map[basic_outcome] == {s: o for (s, o) in outcome_map.iteritems() if s in self._mode_outcome_map[basic_outcome]}:
                    return basic_outcome

            # If we made it past the for loop then none of the outcomes defined in the outcome map were satisfied. So, return the default outcome
            return self._default_outcome
        else:
            return self._default_outcome


    def _mode_child_term_cb(self, outcome_map):
        # Preempt this container if one of the monitor states has terminated
        if len([ms for ms in outcome_map if ms.endswith('_MONITOR') and outcome_map[ms] != None]) > 0:
            return True
        elif self._user_child_term_cb:
            return self._user_child_term_cb(outcome_map)
        # Preempt this container if all of the non-monitor states have finished
        elif len([bs for bs in outcome_map if not bs.endswith('_MONITOR') and outcome_map[bs] == None]) == 0:
            return True
        else:
            return None
