# PI DYSTURBANCE

The Performance Indicator computation for Dysturbance Testbench is implemented in MATLAB.
Further modifications are still needed to be used within the Eurobench Benchmarking Software.

## Purposes

Characterize the balancing performances of robotic subjects under external disturbances.
Technical details on the folder structure are provided in [README](./READ_ME.txt)
More technical details on the functions will be provided within the code [README](src/README.md)

## Usage

We created two scripts to launches the PI computation from the shell of a machine with MATLAB installed.
The two scripts are:
 - `run_Local_PI.bat` launches the computation of the PI for a specific run of the experiments. It requires 3 arguments;
 - `run_Glabal_PI.bat` launches the computation of the PI for a type pf Protocol. It requires 2 arguments;

Assuming the folder `./tests/` contains all the raw data, the shell commands are:

1. LOCAL PI: ARGUMENTS 'csv file name' 'related yaml file name' 'placement folder'
```console
run_Local_PI.bat 'subject_1_cond_1008203500_run_0_platformData.csv' 'subject_1_cond_1008203500_testbed.yaml' 'subject_1/protocol_1'
```
2. GLOBAL PI: ARGUMENTS 'experiment folder' number_of_Protocol
```console
run_Global_PI.bat 'subject_1' 1
```

It is not possible to provide a number of arguments as inputs different from the specified one.
The code will automatically open and run matlab, providing a set of plots and .csv file containing the results of the PI computation. You will find it in the related folder `./tests/subject_1/protocol_1/Global_PI`.


## Acknowledgements

<a href="http://eurobench2020.eu">
  <img src="http://eurobench2020.eu/wp-content/uploads/2018/06/cropped-logoweb.png"
       alt="rosin_logo" height="60" >
</a>

Supported by Eurobench - the European robotic platform for bipedal locomotion benchmarking.
More information: [Eurobench website][eurobench_website]

<img src="http://eurobench2020.eu/wp-content/uploads/2018/02/euflag.png"
     alt="eu_flag" width="100" align="left" >

This project has received funding from the European Union’s Horizon 2020
research and innovation programme under grant agreement no. 779963.

The opinions and arguments expressed reflect only the author‘s view and
reflect in no way the European Commission‘s opinions.
The European Commission is not responsible for any use that may be made
of the information it contains.

[eurobench_logo]: http://eurobench2020.eu/wp-content/uploads/2018/06/cropped-logoweb.png
[eurobench_website]: http://eurobench2020.eu "Go to website"
