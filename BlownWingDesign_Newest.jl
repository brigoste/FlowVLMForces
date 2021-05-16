import FLOWUnsteady
uns = FLOWUnsteady
vlm = uns.vlm
import GeometricTools
gt = GeometricTools
using PyPlot
using LinearAlgebra: norm, dot, cross
using LaTeXStrings                      #Need for setting up plots
using Dates

FLOWUnsteady.vlm.VLMSolver._mute_warning(true)
#Mute the warnings

# velocities = [];       # Total Velocity over wing

today = replace(string(Dates.today()),"-" => "")
savefolder = "data"

disp_plot = true

function main(;disp_plot = true)



    #Define aspects of wing. Single, symmetric wing.

    #Forward Swept Wing
    span = 1.0;              #wing span
    aspectratio = 10.0;      #wing aspect ratio
    taperratio = 0.5;        #wing taper ratio
    wingtwist = 4.0;         #wing twist
    wingsweep = -5.0;        #wing sweep in degrees
    wingdihedral = 6.0;      #wing dihedral in degrees
    savepath = "PlotCollective";

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
    #Now we need some reference information: reference velocity, total time, and initial conditions
    Forces_wing = [];      # Total force over wing.
    forces_per_span = [];  # Force per unit span on wing.
    cL_wing = [];          # Coefficient of lift of wing.
    L_wing = [];           # Lift over wing.
    cD_wing = [];          # Coefficint of drag over wing
    D_wing = [];           # Effective drag over wing.
    y2b_vec = [];
    CdCD_vec = [];
    Gamma_vec = [];
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

            if PFIELD.nt!=0 && PFIELD.nt%nsteps_plot==0 && disp_plot
                # figure(figname)


                # Force at each VLM element
                Ftot = uns.calc_aerodynamicforce(mainwing, prev_wing, PFIELD, Vinf, DT,
                                                                rhoinf; t=PFIELD.t)
                # Add Forces to forces vector
                Fx = [v[1] for v in Ftot]
                Fy = [v[2] for v in Ftot]
                Fz = [v[3] for v in Ftot]
                L, D, S = uns.decompose(Ftot, [0,0,1], [-1,0,0]);
                vlm._addsolution(mainwing, "L", L)
                vlm._addsolution(mainwing, "D", D)
                vlm._addsolution(mainwing, "S", S)
                resultant_Force = [sum(Fx),sum(Fy),sum(Fz)]
                push!(Forces_wing, resultant_Force);

                # #Add Lift to lift vector
                push!(L_wing, L);

                # Force per unit span at each VLM element
                ftot = uns.calc_aerodynamicforce(mainwing, prev_wing, PFIELD, Vinf, DT,
                                            rhoinf; t=PFIELD.t, per_unit_span=true);
                fx = [v[1] for v in ftot]
                fy = [v[2] for v in ftot]
                fz = [v[3] for v in ftot]
                l, d, s = uns.decompose(ftot, [0,0,1], [-1,0,0]);
                resultant_force = fx + fy + fz;
                push!(forces_per_span, resultant_force);

                # Lift of the wing
                Lwing = norm(sum(L))
                CLwing = Lwing/(qinf*b^2/ar)
                ClCL = norm.(l) / (Lwing/b)

                # Add cl to vector
                push!(cL_wing, ClCL);

                # Drag of the wing
                Dwing = norm(sum(D))
                CDwing = Dwing/(qinf*b^2/ar)
                CdCD = [sign(dot(this_d, [1,0,0])) for this_d in d].*norm.(d) / (Dwing/b) # Preserves the sign of drag

                # Add Dwing and CDwing to vectors
                push!(D_wing, Dwing)
                push!(cD_wing, CDwing);
                push!(CdCD_vec, CdCD);

                vlm._addsolution(mainwing, "Cl/CL", ClCL)
                vlm._addsolution(mainwing, "Cd/CD", CdCD)

                #May need to pass this.

                y2b = 2*mainwing._ym/b
                # y2b = sum(y2b)             #sums each part of the the forces over the wing.
                push!(y2b_vec, y2b)
                push!(Gamma_vec, mainwing.sol["Gamma"])

                # subplot(221)
                # # plot(web_2yb, web_ClCL, "ok", label="Weber's experimental data")
                # plot(y2b, ClCL, "-", label="FLOWVLM", alpha=0.5)
                # color = clr
                #
                # subplot(222)
                # # plot(web_2yb, web_CdCD, "ok", label="Weber's experimental data")
                # plot(y2b, CdCD, "-", label="FLOWVLM", alpha=0.5, color=clr)
                #
                # subplot(223)
                # # plot([0, T], web_CL*ones(2), ":k", label="Weber's experimental data")
                # plot([T], [CLwing], "o", label="FLOWVLM", alpha=0.5, color=clr)
                #
                # subplot(224)
                # # plot([0, T], web_CD*ones(2), ":k", label="Weber's experimental data")
                # plot([T], [CDwing], "o", label="FLOWVLM", alpha=0.5, color=clr)
                #
                # figure(figname*"_2")
                # subplot(121)
                # plot(y2b, mainwing.sol["Gamma"], "-", label="FLOWVLM", alpha=0.5, color=clr)
                # if wake_coupled && PFIELD.nt!=0
                #     subplot(122)
                #     plot(y2b, norm.(mainwing.sol["Vkin"])/magVinf, "-", label="FLOWVLM", alpha=0.5, color=[clr[1], 1, clr[3]])
                #     if VehicleType==uns.VLMVehicle
                #         plot(y2b, norm.(mainwing.sol["Vvpm"]), "-", label="FLOWVLM", alpha=0.5, color=clr)
                #     end
                #     plot(y2b, [norm(Vinf(vlm.getControlPoint(mainwing, i), T)) for i in 1:vlm.get_m(mainwing)],
                #                                                 "-k", label="FLOWVLM", alpha=0.5)
                # end
                #
                # figure(figname*"_3")
                # subplot(121)
                # plot([sim.t], [sim.vehicle.V[1]], ".r", label=L"V_x", alpha=0.5)
                # plot([sim.t], [sim.vehicle.V[2]], ".g", label=L"V_y", alpha=0.5)
                # plot([sim.t], [sim.vehicle.V[3]], ".b", label=L"V_z", alpha=0.5)
                # if PFIELD.nt==1; legend(loc="best", frameon=false); end;
                # subplot(122)
                # plot([sim.t], [sim.vehicle.W[1]], ".r", label=L"\Omega_x", alpha=0.5)
                # plot([sim.t], [sim.vehicle.W[2]], ".g", label=L"\Omega_y", alpha=0.5)
                # plot([sim.t], [sim.vehicle.W[3]], ".b", label=L"\Omega_z", alpha=0.5)
                # if PFIELD.nt==1; legend(loc="best", frameon=false); end;
                #
                # #ADDED CODE HERE.
                # #The forces aren't coming through, and I don't know why.
                #
                # #This Plot has a problem
                # figure(figname*"_4")
                # subplot(311)
                # println(sim.t, sum(Fx))
                # scatter(sim.t, sum(Fx), label="Lift forces on wing", alpha = 0.5, color = clr)
                # subplot(312)
                # scatter(sim.t, sum(Fy), label="Drag forces on wing", alpha = 0.5, color = clr)
                # subplot(313)
                # scatter(sim.t, sum(Fz), label="Side forces on wing", alpha = 0.5, color = clr)
                #
                #
                # #All works here except for the last ylabel. It doesn't show up.
                # figure(figname*"_5")
                # subplot(311)
                # plot(y2b, fx, label = "Lift per unit span", alpha = 0.5, color = clr)
                # subplot(312)
                # plot(y2b, fy, label = "Drag per unit span", alpha = 0.5, color = clr)
                # subplot(313)
                # plot(y2b, fz, label = "Side per unit span", alpha = 0.5, color = clr)
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
    #                                     paraview=false,
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
                                        extra_runtime_function=monitor
                                )

    #run(`paraview --data="$save_path/$(files);tutorial_pfield...vtk"`)

    dt = 1/nsteps;
    time_vec = dt:dt:1;

    return cD_wing,cL_wing,D_wing,L_wing,time_vec,Forces_wing,forces_per_span, y2b_vec, CdCD_vec, mainwing, wake_coupled, Gamma_vec
end

function plot_Data(cD_wing, cL_wing, D_wing, L_wing, time_vec, Forces_wing,
    forces_per_span, y2b_vec, CdCD_vec, mainwing, wake_coupled, Gamma_vec, figname = "monitor_plot",
    figsize_factor = 1)
    #Goal, put all plotting here. clear from main.
    #make sure all variables you need are sent here as well.
    #Reset axis on Force plot to start from 0 and run to 1.1 * maximum(vector) on y

    #figure out how to implement colored points

    #Put the save paths in here as well.

    figure(figname, figsize=[7*2, 5*2]*figsize_factor)
    cla()
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
    ylim([0,10]);

    subplot(224)
    xlabel("Simulation time (s)")
    ylabel(L"Drag Coefficient $C_D$")
    ylim([0,0.1]);

    figure(figname*"_2", figsize=[7*2, 5*1]*figsize_factor)
    cla()
    subplot(121)
    xlabel(L"$\frac{2y}{b}$")
    ylabel(L"Circulation $\Gamma$")
    subplot(122)
    xlabel(L"$\frac{2y}{b}$")
    ylabel(L"Effective velocity $V_\infty$")
    #
    # figure(figname*"_3", figsize=[7*2, 5*1]*figsize_factor)
    # cla()
    # subplot(121)
    # xlabel("Simulation time")
    # ylabel("Velocity")
    # subplot(122)
    # xlabel("Simulation time")
    # ylabel("Angular velocity")
    #
    # figure(figname*"_4", figsize=[7*2, 5*1]*figsize_factor)
    # cla()
    # subplot(311)
    # ylabel("Lift Force")
    # subplot(312)
    # ylabel("Drag Force")
    # subplot(313)
    # xlabel("Simulation time")
    # ylabel("Side Force")
    #
    # figure(figname*"_5", figsize = [7*2, 5*1]*figsize_factor)
    # cla()
    # subplot(311)
    # ylabel("Lift per unit span")
    # subplot(312)
    # ylabel("Drag per unit span")
    # subplot(313)
    # xlabel("Simulation time")
    # ylabel("Side per unit span")

    #PLOTS ARE NOW MADE, LETS ADD THE POINTS

    #Problem and fix
    # Basically, time is a 301 unit vector going from 0 to 1. These 301 units show 300 intervals of time
    # Each force is evaulated on the time interval, not the actual time. So, in order to make the vectors the
    #   same length, I push a 0 to the first bucket of each force vector. Make sure this is ok.

    #We are stuck on the third subplot

    #Plot data on figure
    size = length(y2b_vec[1]);

    figure(figname)
    subplot(221)
    for i = 1:size
        aux = (i/size);
        clr = (1-aux, 0, aux)
        plot(y2b_vec[i], cL_wing[i], "-", label="FLOWVLM", alpha=0.5, color = clr)
    end

    subplot(222)
    for i = 1:size
        aux = (i/size);
        clr = (1-aux, 0, aux)
        plot(y2b_vec[i], CdCD_vec[i], "-", label="FLOWVLM", alpha=0.5, color = clr)
    end

    subplot(223)
    size = length(time_vec)
    for i = 1:size
        aux = (i/size);
        # clr = (1-aux, aux, 1+aux)
        clr = (aux, 0, 1-aux)
        # println((time_vec[i]))
        # println(norm(sum(cL_wing[:,i])))
        plot(time_vec[i], norm(sum(cL_wing[i,:])), "o", label="FLOWVLM", alpha=0.5, color=clr)
    end

    subplot(224)
    for i = 1:size
        aux = (i/size);
        clr = (aux, 0, 1-aux);
        plot(time_vec[i], norm(sum(cD_wing[i,:])), "o", label="FLOWVLM", alpha=0.5, color = clr)
    end


    # "Graphs 1 and 2 work well (fix colors), 3 and 4 need work."

    #Problem with this figure.
    # println(size(y2b_vec))                #size = 300
    # println(size(mainwing.sol["Gamma"]))  #size = 40
    #
    # println(length(y2b_vec[1]))
    # println(length(mainwing.sol["Gamma"]))
    # println(mainwing.sol["Gamma"])

    size = length(Gamma_vec[1])
    "y2b_vec"
    println("y2b_vec: ", length(y2b_vec))
    "1 Set of Gamma_vec"
    println("1 set of Gamma_vec: ", length(Gamma_vec))
    figure(figname*"_2")
    subplot(121)
    #Problem with Graphing here.

    for i = 1:size
        plot(y2b_vec, Gamma_vec[:,i], "-", label="FLOWVLM", alpha=0.5)
    end

    # println("Passed Figure 5")
    #add color = clr
    #Figure this one out. We may only need the wake_coupled part, which may need to be passed.
    if wake_coupled
        subplot(122)
        for i = 1:size
            plot(y2b_vec, norm.(mainwing.sol["Vkin"])/magVinf, "-", label="FLOWVLM", alpha=0.5)
            #add color = [clr[1], 1, clr[3]]
            if VehicleType==uns.VLMVehicle
                plot(y2b_vec, norm.(mainwing.sol["Vvpm"]), "-", label="FLOWVLM", alpha=0.5)
                #add color = clr
            end
            plot(y2b_vec, [norm(Vinf(vlm.getControlPoint(mainwing, i), T)) for i in 1:vlm.get_m(mainwing)],
                                                        "-k", label="FLOWVLM", alpha=0.5)
        end
    end
    #
    # figure(figname*"_3")
    # subplot(121)
    # plot([sim.t], [sim.vehicle.V[1]], ".r", label=L"V_x", alpha=0.5)
    # plot([sim.t], [sim.vehicle.V[2]], ".g", label=L"V_y", alpha=0.5)
    # plot([sim.t], [sim.vehicle.V[3]], ".b", label=L"V_z", alpha=0.5)
    # if PFIELD.nt==1; legend(loc="best", frameon=false); end;
    # subplot(122)
    # plot([sim.t], [sim.vehicle.W[1]], ".r", label=L"\Omega_x", alpha=0.5)
    # plot([sim.t], [sim.vehicle.W[2]], ".g", label=L"\Omega_y", alpha=0.5)
    # plot([sim.t], [sim.vehicle.W[3]], ".b", label=L"\Omega_z", alpha=0.5)
    # if PFIELD.nt==1; legend(loc="best", frameon=false); end;
    #
    # figure(figname*"_4")
    # subplot(311)
    # println(sim.t, sum(Fx))
    # scatter(sim.t, sum(Fx), label="Lift forces on wing", alpha = 0.5)
    # #add color = clr
    # subplot(312)
    # scatter(sim.t, sum(Fy), label="Drag forces on wing", alpha = 0.5)
    # #add color = clr
    # subplot(313)
    # scatter(sim.t, sum(Fz), label="Side forces on wing", alpha = 0.5)
    # #add color = clr
    #
    # figure(figname*"_5")
    # subplot(311)
    # plot(y2b_vec, fx, label = "Lift per unit span", alpha = 0.5)
    # #add color = clr
    # subplot(312)
    # plot(y2b_vec, fy, label = "Drag per unit span", alpha = 0.5)
    # #add color = clr
    # subplot(313)
    # plot(y2b_vec, fz, label = "Side per unit span", alpha = 0.5)
    # #add color = clr
    #
    # #Set save file directory
    #
    # topdirectory = normpath(joinpath(@__DIR__)) #Make sure the path is set for inside of the current folder.
    # savepath = joinpath(topdirectory,savepath)  #add a folder into the current path to save the figures.
    # if !(isdir(savepath)); mkdir(savepath); end     #Make sure that savepath is there. Makes one if not
    #
    # figname = "monitor_plot"
    #
    # figure(figname)
    # savefig(joinpath(savepath, figname*"_"*today*".pdf"))
    #
    # figure(figname*"_2")
    # savefig(joinpath(savepath,figname*"_2_"*today*".pdf"))
    #
    # figure(figname*"_3")
    # savefig(joinpath(savepath,figname*"_3_"*today*".pdf"))
    #
    # figure(figname*"_4")
    # savefig(joinpath(savepath, figname*"_4_"*today*".pdf"))
    #
    # figure(figname*"_5")
    # savefig(joinpath(savepath,figname*"_5_"*today*".pdf"))
end


cD_wing,cL_wing,D_wing,L_wing,time_vec,Forces_wing,forces_per_span, y2b_vec, CdCD_vec, mainwing, wake_coupled, Gamma_vec = main()

if disp_plot == true
    plot_Data(cD_wing, cL_wing, D_wing, L_wing, time_vec, Forces_wing, forces_per_span, y2b_vec, CdCD_vec, mainwing, wake_coupled, Gamma_vec)
end
