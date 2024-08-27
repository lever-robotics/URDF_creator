import React from 'react';
import './page3.css';

/**
 * Page3 component that explains how to manipulate shapes in the URDF Creator tool.
 * 
 * @returns {JSX.Element} The rendered component.
 */
const Page3 = () => {
    return (
        <div className="page3-container">
            <div className="graphic-container">
                <img src={process.env.PUBLIC_URL + '/statics/box.png'} alt="Add Cube" className="graphic"/>
            </div>

            <h2 className="section-header">Manipulating Shapes</h2>
            <p className="description">
                Manipulate the cube by first selecting it either by selecting it or clicking on it in the Object Tree.
            </p>

            <div className="graphic-container">
                <img src={process.env.PUBLIC_URL + '/statics/translate.png'} alt="Translation" className="graphic"/>
            </div>
            <p className="description">
                Move the cube accordingly to the shape of the base of the robot off the ground as high as the wheels will be.
            </p>

            <div className="graphic-container">
                <img src={process.env.PUBLIC_URL + '/statics/scale.png'} alt="Scale" className="graphic"/>
            </div>
            <p className="description">
                To give specific scaling, positioning, or rotation of a link, use the parameters in the Object Parameters section.
            </p>

            <div className="graphic-container">
                <img src={process.env.PUBLIC_URL + '/statics/params.png'} alt="Object Parameters" className="graphic"/>
            </div>
            <p className="description">
                We can use the duplicate function by right-clicking on the link in the Object Tree and selecting duplicate. This is also a way to delete links.
            </p>

            <div className="graphic-container">
                <img src={process.env.PUBLIC_URL + '/statics/duplicate.png'} alt="Duplicate" className="graphic"/>
            </div>
        </div>
    );
};

export default Page3;
