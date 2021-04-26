import FLOWUnsteady
uns = FLOWUnsteady
vlm = uns.vlm
import GeometricTools
gt = GeometricTools
using PyPlot
using LinearAlgebra: norm, dot, cross
using LaTeXStrings                      #Need for setting up plots

# velocities = [];       # Total Velocity over wing




function main(;disp_plot = true)



    #Define aspects of wing. Single, symmetric wing.

    #Forward Swept Wing
    span = 1.0;              #wing span
    aspectratio = 10.0;      #wing aspect ratio
    taperratio = 0.5;        #wing taper ratio
    wingtwist = 4.0;         #wing twist
    wingsweep = -5.0;        #wing sweep in degrees
    wingdihedral = 6.0;      #wing dihedral in degrees

    #Glider
    # span = 1.0;
    # aspectratio = 15.0;
    # taperratio = 0.9;
    # wingtwist = 1.0;
    # wingsweep = 1.0;
    # wingdihedral = 1.0;

    #This bulids the wing using the variables we gave it.
    mainwing = vlm.simpleWing(span,aspectratio,taperratio,wingtwist,wingsweep,wingdihedral);

    #Now we create a wing system
    system = vlm.WingSystem();       #creates an empty system

    vlm.addwing(system, "mainwing", mainwing);  #Adds wing to wystem

    #We will now save the system to a .vtk file. MAKE SURE YOU ARE IN A NON-IMPORANT LOCATION TO SAVE THIS.
    #I added an empty folder for this.

    #Prep file
    Vinf(X,t) = [1,0,0];        #non-dimensional function defining free stream velocity
    vlm.setVinf(system, Vinf)   #set freestream velocity for the system
    "Freestream Velocity set"
    #"$Vinf"

    run_name = "tutorial"           #define identifier at beginning of file names
    save_path = "./simplewing/"     #define directory where files will be saved

    run(`rm -rf $save_path`)        #clear out directory where files will be saved
    run(`mkdir $save_path`)         #re-create directory fresh

    vlm.save(system, run_name; path=save_path)  #save geometry in a .vtk file format

    #now we will show the wing using paraview

    # run(`paraview --data="$(save_path)/$(run_name)_mainwing_vlm.vtk"`)

    #We now have the wing set up and running in ParaView.

    #We will now add a rotor
    rotor_file = "apc10x7.csv"          # Rotor geometry
    data_path = uns.def_data_path       # Path to rotor database

    #this creates a rotor, and generates a heck-a-ton of output, which we suppress.
    R, B = uns.read_rotor(rotor_file; data_path=data_path)[[1,3]] #get the radius for later

    Vinf(X,t) = [1,0,0];
    # Vinf = [1,0,0];

    rotor = uns.generate_rotor(rotor_file; pitch=0.0,
                                            n=10, CW=true, ReD=1.5e6,
                                            verbose=true, xfoil=false,
                                            data_path=data_path,
                                            plot_disc=false);

    #Create a rotor object bsed on parameters above.
    rotors = vlm.Rotor[rotor];
    #We now set the location of the vehicle to be in line with the rotor.

    #First we set the vehicle at the origin.
    vehicleorigin = [0.0; 0.0; 0.0];
    vehicleaxis = [1.0 0.0 0.0; 0.0 1.0 0.0; 0.0 0.0 1.0];

    #Now we set the rotor origin in line with the vehicle
    rotororigin = [-0.1; 0.0; 0.0];
    RPM_Val = 1000

    #And now line the rest of the rotor up, part by part.
    for rotor in rotors
        vlm.setcoordsystem(rotor, rotororigin, vehicleaxis; user=true);
        vlm.setRPM(rotor, RPM_Val)
    end

    #Now we will store the rotor in a tuple
    rotor_systems = (rotors,);
    # vlm.setRPM(rotors, RPM);       #Maybe this.

    #finally, we add the rotor to the wing system. Again, part by part.
    for rotor in rotors
        vlm.addwing(system, run_name, rotor);
    end

    #Now we run it to see the visualization
    run(`rm -rf $save_path`)
    run(`mkdir $save_path`)
    #
    vlm.setVinf(system, Vinf)

    # vlm.setRPM(system, RPM);

    vlm.save(system, run_name; path=save_path)
    # run(`paraview --data="$(save_path)/tutorial_mainwing_vlm.vtk;tutorial_tutorial_Blade1_vlm.vtk;tutorial_tutorial_Blade2_vlm.vtk;tutorial_tutorial_Blade1_loft.vtk;tutorial_tutorial_Blade2_loft.vtk;"`)

    #Starting the new code block. There is still problems in the above code.


    ######################### Unrun code. #######################################
    #Should work, as it is basically line for line from online

    #create a vlm system to add our main wing to.
    vlm_system = vlm.WingSystem();
    vlm.addwing(vlm_system, "mainwing", mainwing);

    #Create a wake system. Add VLM system and rotor to this wake system.
    wake_system = vlm.WingSystem();
    # vlm.addwing(wake_system, "SolveVlm", vlm_system)

    # for rotor in rotors
    #     vlm.addwing(wake_system, run_name, rotor);
    # end

    #We need to define tilting objects (as they are part of the code) but ours will be empty

    tilting_systems = ();   #Empty tuple

    #The vehicle, rotor, and freestream are all set up now.

    #We will now define a maneuver.

    #Start with Vehicle velocity function
    Vvehicle(t) = [-1.0,0.0,0.0];

    #Define Vehicle angle
    anglevehicle(t) = zeros(3); #Flat angle

    #define angle of tilting system. We have no tilting system, so make it empty
    angle = ();

    #Now, lastly, we define the rotation rate function for the rotors
    RPM_fun(t) = 10.0;   #1.0 is constant rotation
    RPM = (RPM_fun,);

    #using each individual element, we can create a maneuver now.
    maneuver = uns.KinematicManeuver(angle, RPM, Vvehicle, anglevehicle);

    #We can plot the maneuver now, which is good.
    uns.plot_maneuver(maneuver);

    #This is cool, but lets put it in paraview.
    vlm.setVinf(system, Vinf);
    vehicle = uns.QVLMVehicle( system;
                              tilting_systems = tilting_systems,
                              rotor_systems = rotor_systems,
                              vlm_system = vlm_system,
                              wake_system = wake_system,
                            );
    #Now we need some reference information: referenc velocity, total time, and initial conditions
    Forces_wing = [];      # Total force over wing.        Done, line 70
    forces_per_span = [];  # Force per unit span on wing.  Done, line 82
    cL_wing = [];          # Coefficient of lift of wing.  Done, line 87
    L_wing = [];           # Lift over wing.               Done, line 76
    cD_wing = [];          # Coefficint of drag over wing. Done Line 107
    D_wing = [];           # Effective drage over wing.    Done Line 106
    dt = 0;
    Time = 0;
    nsteps = 300;

    Flag = false;           #testing for the forces output

    Vref = 10.0         #define a reference velocity for the vehicle
    ttot = 1.0          #define a total simulation time, in seconds
    nsteps = 300        #define the number of steps the simulation will take
    prev_wing = deepcopy(mainwing)
    rhoinf = 1.23
    b = span
    qinf = 0.5*Vref^2 * rhoinf
    ar = aspectratio
    wake_coupled = false
    function monitor(sim, PFIELD, T, DT; figname="monitor_plot", nsteps_plot=1, disp_plot = disp_plot, figsize_factor = 1)
        #We need to write the monitor function, as seen in the examples, such the heavingwing example.

        #WRITE YOUR OWN MONITOR FUNCTION, as it is specific to your simulation.

        #To get the forces/moments/lift/lift coefficitnets, either declare variables outside of the function that
        #   you add to with the push!() command each iteration, or print all the numbers to a text file which you
        #   can use later. This is how you get these values for the wing.

        #Example of monitor function starts on line 199 of heavingwing.jl

        #There is a lot going on in here, especially a lot of plotting. Do you need this?
        #Maybe deleate anything you don't need to find the Forces and coefficients.

        aux = PFIELD.nt/nsteps
            clr = (1-aux, 0, aux)

            if PFIELD.nt==0 && disp_plot
                figure(figname, figsize=[7*2, 5*2]*figsize_factor)
                subplot(221)
                xlim([0,1])
                xlabel(L"$\frac{2y}{b}$")
                ylabel(L"$\frac{Cl}{CL}$")
                title("Spanwise lift distribution")

                subplot(222)
                xlim([0,1])
                xlabel(L"$\frac{2y}{b}$")
                ylabel(L"$\frac{Cd}{CD}$")
                title("Spanwise drag distribution")

                subplot(223)
                xlabel("Simulation time (s)")
                ylabel(L"Lift Coefficient $C_L$")
                ylim([0,1]);



                subplot(224)
                xlabel("Simulation time (s)")
                ylabel(L"Drag Coefficient $C_D$")
                ylim([0,0.1]);

                figure(figname*"_2", figsize=[7*2, 5*1]*figsize_factor)
                subplot(121)
                xlabel(L"$\frac{2y}{b}$")
                ylabel(L"Circulation $\Gamma$")
                subplot(122)
                xlabel(L"$\frac{2y}{b}$")
                ylabel(L"Effective velocity $V_\infty$")

                figure(figname*"_3", figsize=[7*2, 5*1]*figsize_factor)
                subplot(121)
                xlabel("Simulation time")
                ylabel("Velocity")
                subplot(122)
                xlabel("Simulation time")
                ylabel("Angular velocity")

                figure(figname*"_4", figsize=[7*2, 5*1]*figsize_factor)
                subplot(121)
                xlabel("Simulation time")
                ylabel("Total Force")
                subplot(122)
                xlabel("Simulation time")
                ylabel("Force per unit span")

            end

            if PFIELD.nt!=0 && PFIELD.nt%nsteps_plot==0 && disp_plot
                figure(figname)


                # Force at each VLM element
                Ftot = uns.calc_aerodynamicforce(mainwing, prev_wing, PFIELD, Vinf, DT,
                                                                rhoinf; t=PFIELD.t)
                # Add Forces to forces vector
                resultant_Force = sqrt(Ftot[1]^2 + Ftot[2]^2 + Ftot[3]^2);  #Resultnat vector
                push!(Forces_wing, restultant_Force);
                L, D, S = uns.decompose(Ftot, [0,0,1], [-1,0,0]);
                vlm._addsolution(mainwing, "L", L)
                vlm._addsolution(mainwing, "D", D)
                vlm._addsolution(mainwing, "S", S)

                # #Add Lift to lift vector
                push!(L_wing, L);

                # Force per unit span at each VLM element
                ftot = uns.calc_aerodynamicforce(mainwing, prev_wing, PFIELD, Vinf, DT,
                                            rhoinf; t=PFIELD.t, per_unit_span=true);
                l, d, s = uns.decompose(ftot, [0,0,1], [-1,0,0]);

                # Add Force/span to f vector
                resultant_force = sqrt(ftot[1]^2 + ftot[2]^2 + ftot[3]^2);
                push!(forces_per_span, resultant_force);
                # Lift of the wing
                Lwing = norm(sum(L))
                CLwing = Lwing/(qinf*b^2/ar)
                ClCL = norm.(l) / (Lwing/b)

                # Add cl to vector
                push!(cL_wing, CLwing);

                # Drag of the wing
                Dwing = norm(sum(D))
                CDwing = Dwing/(qinf*b^2/ar)
                CdCD = [sign(dot(this_d, [1,0,0])) for this_d in d].*norm.(d) / (Dwing/b) # Preserves the sign of drag

                # Add Dwing and CDwing to vectors
                push!(D_wing, Dwing)
                push!(cD_wing, CDwing);

                vlm._addsolution(mainwing, "Cl/CL", ClCL)
                vlm._addsolution(mainwing, "Cd/CD", CdCD)

                y2b = 2*mainwing._ym/b

                subplot(221)
                # plot(web_2yb, web_ClCL, "ok", label="Weber's experimental data")
                plot(y2b, ClCL, "-", label="FLOWVLM", alpha=0.5, color=clr)

                subplot(222)
                # plot(web_2yb, web_CdCD, "ok", label="Weber's experimental data")
                plot(y2b, CdCD, "-", label="FLOWVLM", alpha=0.5, color=clr)

                subplot(223)
                # plot([0, T], web_CL*ones(2), ":k", label="Weber's experimental data")
                plot([T], [CLwing], "o", label="FLOWVLM", alpha=0.5, color=clr)

                subplot(224)
                # plot([0, T], web_CD*ones(2), ":k", label="Weber's experimental data")
                plot([T], [CDwing], "o", label="FLOWVLM", alpha=0.5, color=clr)

                figure(figname*"_2")
                subplot(121)
                plot(y2b, mainwing.sol["Gamma"], "-", label="FLOWVLM", alpha=0.5, color=clr)
                if wake_coupled && PFIELD.nt!=0
                    subplot(122)
                    plot(y2b, norm.(mainwing.sol["Vkin"])/magVinf, "-", label="FLOWVLM", alpha=0.5, color=[clr[1], 1, clr[3]])
                    if VehicleType==uns.VLMVehicle
                        plot(y2b, norm.(mainwing.sol["Vvpm"]), "-", label="FLOWVLM", alpha=0.5, color=clr)
                    end
                    plot(y2b, [norm(Vinf(vlm.getControlPoint(mainwing, i), T)) for i in 1:vlm.get_m(mainwing)],
                                                                "-k", label="FLOWVLM", alpha=0.5)
                end

                figure(figname*"_3")
                subplot(121)
                plot([sim.t], [sim.vehicle.V[1]], ".r", label=L"V_x", alpha=0.5)
                plot([sim.t], [sim.vehicle.V[2]], ".g", label=L"V_y", alpha=0.5)
                plot([sim.t], [sim.vehicle.V[3]], ".b", label=L"V_z", alpha=0.5)
                if PFIELD.nt==1; legend(loc="best", frameon=false); end;
                subplot(122)
                plot([sim.t], [sim.vehicle.W[1]], ".r", label=L"\Omega_x", alpha=0.5)
                plot([sim.t], [sim.vehicle.W[2]], ".g", label=L"\Omega_y", alpha=0.5)
                plot([sim.t], [sim.vehicle.W[3]], ".b", label=L"\Omega_z", alpha=0.5)
                if PFIELD.nt==1; legend(loc="best", frameon=false); end;

                #ADDED CODE HERE.
                #The forces aren't coming through, and I don't know why.

                figure(figname*"_4")
                subplot(121)
                plot([sim.t], resltant_Force, label="Total forces on wing", alpha = 0.5)
                subplot(122)
                plot([sim.t], ftot[1],label = "Force per unit span", alpha = 0.5)

            end

            prev_wing = deepcopy(mainwing)

            return false

    end
    #initial conditions
    tinit = 0.0                                  #initial time
    Vinit = Vref*maneuver.Vvehicle(tinit/ttot)   #initial linear velocity
    Winit = zeros(3)                             #initial angular velocity
    RPMreference = RPM_Val

    #Everything is now defined! Let's create the simulation

    simulation = uns.Simulation(vehicle, maneuver, Vref, RPMreference, ttot; Vinit=Vinit, Winit=Winit, t=tinit);

    #Visulaize the simulation in paraview
    # files = uns.visualize_kinematics(   simulation, nsteps, save_path;
    #                                     run_name=run_name,
    #                                     prompt=false,
    #                                     paraview=false
    #                                 )

    # run(`paraview --data="$save_path/$files"`)

    #The simulation also has some outputs we can recieve. The following code does this.

    # nullfunc(args...) = false
    pfield = uns.run_simulation(simulation, nsteps;
                                        surf_sigma=R/10,
                                        Vinf=Vinf,
                                        save_path=save_path,
                                        run_name=run_name,
                                        prompt=false,
                                        verbose=true,
                                        extra_runtime_function=monitor     #THIS LINE IS WHERE YOU CALL THE FORCES
                                )
                                #See line 325 of heavingwing.jl

    #run(`paraview --data="$save_path/$(files);tutorial_pfield...vtk"`)

    #THIS WORKS

    #Now, we go into the code and pull the lift/drag/and moments from the wing.
    #Plot those over time.

    #Look into the examples on FLOWUnsteady for the function call that returns ALL of those values.

    #We need to write the monitor function, as seen in the examples, such the heavingwing example.

    #WRITE YOUR OWN MONITOR FUNCTION, as it is specific to your simulation.

    #To get the forces/moments/lift/lift coefficitnets, either declare variables outside of the function that
    #   you add to with the push!() command each iteration, or print all the numbers to a text file which you
    #   can use later. This is how you get these values for the wing.

    #Example of monitor function starts on line 199 of heavingwing.jl

    dt = ttot/nsteps;
    time_vec = 0:dt:ttot;

    return cD_wing,cL_wing,D_wing,L_wing,time_vec,Forces_wing, forces_per_span

end

cD_wing,cL_wing,D_wing,L_wing,time_vec, Forces_wing, forces_per_span = main()

#
println(time_vec)
println(Forces_wing)    #This is not being added to. Is it a problem with it as a global variable?
                        #Can Julia push into this array in the function, then print it here?

FirstForce = zeros();            #Empty Tuple

Forces_wing = insert!(Forces_wing,1,FirstForce);     #add azero to the start

println(Forces_wing(1));
#FROM WHAT I UNDERSTNAD, THESE PLOTS ARE DONE, WITHOUT LABELS
#Plots of Forces
#Forces_wing is a very convoluted vector. It is a tuple.
# figure("Forces")
# subplot(211)
# plot(time_vec, Forces_wing[:,1])
# xlabel("Time(s)");
# ylabel("Force")
# subplot(212)
# plot(time_vec, forces_per_span[:,1]);
# xlabel("Time(s)")
# ylabel("Force")
#
# #Plots of lift
# figure("Lift")
# subplot(211)
# plot(time_vec, cl_wing);
# xlabel("time (s)");
# ylabel("cL")
# subplot(212)
# plot(time_vec, L_wing, label = cl);
# xlabel("time(s)")
# ylabel("L")
