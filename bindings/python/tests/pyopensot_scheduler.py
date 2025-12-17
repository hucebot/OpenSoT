import unittest
from pyopensot.oc import *

class TestScheduler(unittest.TestCase):
    
    def setUp(self):
        """Create a basic scheduler for each test"""
        self.sch = Scheduler()
        self.sch.addContact("l", ["Left"])
        self.sch.addContact("r", ["Right"])
        self.sch.addContact("both", ["Left", "Right"])
    
    def test_basic_sequence(self):
        """Test basic sequence sampling from start"""
        self.sch.addPhase(["both"], 1.0)
        self.sch.addPhase(["l"], 1.0)
        
        seq = self.sch.getSequence(0.5, nodes_number=4, current_time=0.0)
        
        expected = [
            ["Left", "Right"],  # t=0.0
            ["Left", "Right"],  # t=0.5
            ["Left"],           # t=1.0
            ["Left"]            # t=1.5
        ]
        
        self.assertEqual(list(seq), expected)
    
    def test_start_mid_sequence(self):
        """Test starting from middle of sequence"""
        self.sch.addPhase(["both"], 1.0)
        self.sch.addPhase(["l"], 1.0)
        
        seq = self.sch.getSequence(0.5, nodes_number=4, current_time=0.7)
        
        expected = [
            ["Left", "Right"],  # t=0.7 (in phase 1)
            ["Left"],           # t=1.2 (in phase 2)
            ["Left"],           # t=1.7 (in phase 2)
            ["Left", "Right"]   # t=2.2 (wrapped to phase 1)
        ]
        
        self.assertEqual(list(seq), expected)
    
    def test_sequence_wrapping(self):
        """Test that sequence wraps around correctly"""
        self.sch.addPhase(["l"], 1.0)
        self.sch.addPhase(["r"], 1.0)
        
        seq = self.sch.getSequence(0.5, nodes_number=6, current_time=0.0)
        
        expected = [
            ["Left"],   # t=0.0
            ["Left"],   # t=0.5
            ["Right"],  # t=1.0
            ["Right"],  # t=1.5
            ["Left"],   # t=2.0 (wrapped)
            ["Left"]    # t=2.5 (wrapped)
        ]
        
        self.assertEqual(list(seq), expected)
    
    def test_start_time_beyond_sequence(self):
        """Test starting time that's beyond one full sequence cycle"""
        self.sch.addPhase(["l"], 1.0)
        self.sch.addPhase(["r"], 1.0)
        
        seq = self.sch.getSequence(0.5, nodes_number=3, current_time=2.3)
        
        expected = [
            ["Left"],   # t=2.3 -> wraps to 0.3 (phase 1)
            ["Left"],   # t=2.8 -> wraps to 0.8 (phase 1)
            ["Right"]   # t=3.3 -> wraps to 1.3 (phase 2)
        ]
        
        self.assertEqual(list(seq), expected)
    
    def test_single_phase(self):
        """Test with only one phase"""
        self.sch.addPhase(["both"], 2.0)
        
        seq = self.sch.getSequence(0.5, nodes_number=5, current_time=0.0)
        
        expected = [["Left", "Right"]] * 5
        
        self.assertEqual(list(seq), expected)
    
    def test_fractional_sampling(self):
        """Test with non-round sampling intervals"""
        self.sch.addPhase(["l"], 1.0)
        self.sch.addPhase(["r"], 1.0)
        
        seq = self.sch.getSequence(0.3, nodes_number=4, current_time=0.0)
        
        expected = [
            ["Left"],   # t=0.0
            ["Left"],   # t=0.3
            ["Left"],   # t=0.6
            ["Left"]    # t=0.9 (still in phase 1)
        ]
        
        self.assertEqual(list(seq), expected)
    
    def test_no_nodes_limit(self):
        """Test without nodes_number specified (one full cycle)"""
        self.sch.addPhase(["l"], 1.0)
        self.sch.addPhase(["r"], 1.0)
        
        seq = self.sch.getSequence(1.0, nodes_number=-1, current_time=0.0)
        
        expected = [
            ["Left"],   # t=0.0
            ["Right"]   # t=1.0
        ]
        
        self.assertEqual(list(seq), expected)
    
    def test_phase_extends_sequence(self):
        """Test that adding phases extends the sequence"""
        self.sch.addPhase(["l"], 0.5)
        seq1 = self.sch.getSequence(0.5, nodes_number=-1, current_time=0.0)
        self.assertEqual(len(list(seq1)), 1)
        
        self.sch.addPhase(["r"], 0.5)
        seq2 = self.sch.getSequence(0.5, nodes_number=-1, current_time=0.0)
        self.assertEqual(len(list(seq2)), 2)
    
    def test_multiple_named_sequences(self):
        """Test multiple independent named sequences"""
        self.sch.addPhase(["l"], 1.0, sequence_name="seq1")
        self.sch.addPhase(["r"], 1.0, sequence_name="seq2")
        
        seq1 = self.sch.getSequence(0.5, sequence_name="seq1", nodes_number=2, current_time=0.0)
        seq2 = self.sch.getSequence(0.5, sequence_name="seq2", nodes_number=2, current_time=0.0)
        
        expected1 = [["Left"], ["Left"]]
        expected2 = [["Right"], ["Right"]]
        
        self.assertEqual(list(seq1), expected1)
        self.assertEqual(list(seq2), expected2)


if __name__ == '__main__':
    unittest.main()