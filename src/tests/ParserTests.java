package tests;
import internal.Helper.Vector;
import org.junit.Test;

import internal.Helper.Parser;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

import org.junit.Before;
/**
 * 
 * @author Anthony Rathe
 *
 */
public class ParserTests {
	
	Parser parser;
	String path;
	
	@Before
	public void setup() {
		path = "src/internal/blockCoordinates.txt";
		parser = new Parser(path);
	}
	
	@Test
	public void multipleLinesOfFloatsTest() throws IOException {
		parser.getCoordinates();
	}

	@Test
	public void sortTest(){
		List<Vector> vectorList = new ArrayList<>();
		vectorList.add(new Vector(1,0,0));
		vectorList.add(new Vector(0,2,0));
		vectorList.add(new Vector(0,0,3));

		vectorList.sort(new Comparator<Vector>() {
			@Override
			public int compare(Vector v1, Vector v2) {
				if (v1.getSize() < v2.getSize()) {
					return 1;
				} else if (v1.getSize() > v2.getSize()) {
					return -1;
				} else {
					return 0;
				}
			}
		});

		System.out.println(vectorList);
	}
	
}
