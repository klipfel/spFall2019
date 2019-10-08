clear all;close all;
fprintf("Simulation of the observer.\n");
obs = observer("Niter", 100);
%obs = observer();
obs.simulation();
intropsector = introspection(obs);
intropsector.introspect(intropsector.conf.full);
%obs.delete();
