# PI DYSTURBANCE
The Performance Indicator computation for Dysturbance Testbench is implemented in MATLAB.
Further modifications are still needed to be used within the Eurobench Benchmarking Software.

## Purposes
Characterize the balancing performances of robotic subjects under external disturbances.
Technical details on the folder structure are provided in [src/README.md](src/README.md)

## Usage
There are two distinct scripts for the PI computation of the DYSTURBANCE testbed:
1. `run_local_pi` computes the PI for a specific run on a series of experiments on a given subject.

   Args (all required):
   1. an input `.csv` file containing the experiment data;
   2. an input `.yaml` file containing the experiment info (testbed, subject, etc...)
   3. an output directory where the results is stored

2. `run_global_pi` computes the PI for a given protocol on the whole series of experiments on a given subject.

   Args (all required):
    1. an input directory containing all the data of the tested subject
    2. an integer which identifies the protocol to be evaluated (1-5)
    3. an output directory where the results is stored

Assuming folder `./tests/data/input` contains all the raw input data, and that `./tests/data/output` exists and will contain the resulting files, the shell command examples are:
1. ```console
   ./run_local_pi ./tests/data/input/subject_1/protocol_1/subject_1_cond_1008203500/raw_data_input/subject_1_cond_1008203500_run_0_platformData.csv ./tests/data/input/subject_1/protocol_1/subject_1_cond_1008203500/raw_data_input/subject_1_cond_1008203500_testbed.yaml ./tests/data/output/
   ```
2. ```console
   ./run_global_pi ./tests/data/input/subject_1/ 1 ./tests/data/output/
   ```

It is not possible to provide a number of arguments as inputs different from the specified one.

The code will automatically open and run MATLAB, providing a set of plots and .csv file containing the results of the PI computation.

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