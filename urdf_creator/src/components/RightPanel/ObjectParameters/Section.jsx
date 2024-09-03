import React, { useState } from "react";

const Section = ({ title, children }) => {

    return (
        <div>
            <h4>{title}</h4>
            <div className="section">
                {children}
            </div>
            {/* <hr className="line"></hr> */}
        </div>
    );
};

export default Section;