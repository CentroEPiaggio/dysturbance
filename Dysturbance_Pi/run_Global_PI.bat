@echo off 
echo ARGOMENTI %1 %2
matlab -nodisplay -r "run_Global_PI(%1, %2);exit"