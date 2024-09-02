import React, { useState } from "react";

const Section = ({ title, children }) => {

    return (
        <>
            <h4>{title}</h4>
            <div className="section">
                {children}
            </div>
        </>
    );
};

export default Section;