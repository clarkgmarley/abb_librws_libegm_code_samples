MODULE TRobMain
!======================================================================================================
! Copyright (c) 2018, ABB Schweiz AG
! All rights reserved.
!
! Redistribution and use in source and binary forms, with
! or without modification, are permitted provided that
! the following conditions are met:
!
!    * Redistributions of source code must retain the
!      above copyright notice, this list of conditions
!      and the following disclaimer.
!    * Redistributions in binary form must reproduce the
!      above copyright notice, this list of conditions
!      and the following disclaimer in the documentation
!      and/or other materials provided with the
!      distribution.
!    * Neither the name of ABB nor the names of its
!      contributors may be used to endorse or promote
!      products derived from this software without
!      specific prior wrSitten permission.
!
! THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
! AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
! IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
! ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
! LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
! DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
! SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
! CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
! OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
! THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
!======================================================================================================
    
    !***********************************************************
    ! Program data
    !***********************************************************
    RECORD TestCustomRecord
          num counter;
          jointtarget joint_target;
          robtarget rob_target;
    ENDRECORD
    
    VAR num test_num := 0;
    VAR dnum test_dnum := 0;
    VAR bool test_bool := false;
    VAR string test_string := "";
    VAR pose test_pose := [[0,0,0], [1,0,0,0]];
    VAR TestCustomRecord test_custom_record;
    
    !***********************************************************
    !
    ! Procedure main
    !
    !***********************************************************
    PROC main()
        TPErase;
        TPWrite "RAPID started";
        test_custom_record.counter := 0;
        test_custom_record.joint_target := CJointT();
        test_custom_record.rob_target := CRobT();
        
        WHILE TRUE DO
            TPWrite "RAPID is running...";

            ! Toogle 'TEST_IO_SINGAL_2'
            SetDo TEST_IO_SIGNAL_2, LOW;
            WaitTime 1.0;
            SetDo TEST_IO_SIGNAL_2, HIGH;
            WaitTime 1.0;
        ENDWHILE
    ENDPROC
ENDMODULE