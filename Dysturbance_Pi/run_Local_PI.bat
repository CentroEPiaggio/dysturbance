@echo off 
echo ARGOMENTI %1 %2 %3
matlab -nodisplay -r "run_Local_PI(%1, %2, %3);exit"