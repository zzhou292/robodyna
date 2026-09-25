"""Source ABI parsing checks; these do not execute or simulate a native routine."""
import unittest
from .abi import arguments, verify_common_prefix


class NativeAbi(unittest.TestCase):
    def test_fixed_form_continuations_and_comment_decoys(self):
        text='C     SUBROUTINE FRAME(FAKE)\n      SUBROUTINE FRAME(X,\n     A A,STATE_1)\n      RETURN\n'
        self.assertEqual(arguments(text,'frame'),('X','A','STATE_1'))

    def test_ambiguous_or_missing_signatures_reject(self):
        for text in ('      SUBROUTINE FRAME(A,A)\n','      SUBROUTINE FRAME(A)\n'*2,
                     '      SUBROUTINE DIFFERENT(A)\n'):
            with self.assertRaises(ValueError):arguments(text,'FRAME')

    def test_only_verified_initial_scalar_common_fields_are_admitted(self):
        text='      COMMON /COM/ A,B,\n     . C,ARRAY(10),D\n      INTEGER A,B,C,ARRAY,D\n'
        verify_common_prefix(text,'COM',('A','B','C'))
        for expected in (('A','C'),('A','B','C','D')):
            with self.assertRaises(ValueError):verify_common_prefix(text,'COM',expected)


if __name__=='__main__':unittest.main()
