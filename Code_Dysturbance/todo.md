# Needed code evolution

## Figure plotting

The front-end is likely to provide visualizations of the PI.
The current code is also generating some figures.
As the processing will be done in batch, the creation of the figure is questionnable.

Open question:

* would it makes sense to store with the PI a set of figures (with an image format), that could be displayed for the user (as is), together with some general image description?
