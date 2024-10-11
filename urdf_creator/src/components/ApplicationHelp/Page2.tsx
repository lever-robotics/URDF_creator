import React from "react";
import "./onboarding.css";

/**
 * Page2 component that explains URDF and its components like Links and Joints.
 *
 * @returns {JSX.Element} The rendered component.
 */
const Page2 = () => {
    return (
        <div className="page-container">
            <h2 className="section-header">
                What is a <span className="blue">URDF</span>?
            </h2>
            <p className="description">
                A URDF (Uniform Robot Description Format) serves as a universal
                XML-based format for describing robots. It describes its
                structure and kinematics.
            </p>
            <div className="content-section">
                <div className="text-section">
                    <h3 className="subsection-header">Links</h3>
                    <p className="description">
                        <strong className="blue">Links</strong> represent
                        physical components of the robot, such as wheels, arms,
                        or sensors.
                    </p>
                </div>
                <div className="graphic-container-page2">
                    <img
                        src={"/statics/link.png"}
                        alt="Links"
                        className="graphic"
                    />
                </div>
            </div>

            <div className="content-section">
                <div className="text-section">
                    <h3 className="subsection-header">Joints</h3>
                    <p className="description">
                        <strong className="blue">Joints</strong> define the
                        connections between these links, enabling movement and
                        articulation.
                    </p>
                </div>
                <div className="graphic-container-page2">
                    <img
                        src={"/statics/joints.png"}
                        alt="Joints"
                        className="graphic"
                    />
                </div>
            </div>
        </div>
    );
};

export default Page2;
