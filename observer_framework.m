clear all;close all;
fprintf("Simulation of the observer.\n");
obs = observer("Niter", 1);
obs.simulation();
intropsector = introspection(obs);
intropsector.introspect(intropsector.conf.full);
%obs.delete();
